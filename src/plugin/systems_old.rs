//! Systems responsible for interfacing our Bevy components with the Rapier physics engine.

use crate::control::CharacterCollision;
use crate::dynamics::{
    AdditionalMassProperties, Ccd, Damping, Dominance, ExternalForce, ExternalImpulse,
    GravityScale, ImpulseJoint, LockedAxes, MassProperties, MultibodyJoint,
    RapierImpulseJointHandle, RapierMultibodyJointHandle, RapierRigidBodyHandle,
    ReadMassProperties, RigidBody, Sleeping, TransformInterpolation, Velocity,
};
use crate::geometry::{
    ActiveCollisionTypes, ActiveEvents, ActiveHooks, Collider, ColliderDisabled,
    ColliderMassProperties, ColliderScale, CollisionGroups, ContactForceEventThreshold, Friction,
    RapierColliderHandle, Restitution, Sensor, SolverGroups,
};
use crate::pipeline::{CollisionEvent, ContactForceEvent};
use crate::plugin::configuration::{SimulationToRenderTime, TimestepMode};
use crate::plugin::{RapierConfiguration, RapierContext};
use crate::prelude::{
    AdditionalSolverIterations, BevyPhysicsHooks, BevyPhysicsHooksAdapter, CollidingEntities,
    ContactSkin, KinematicCharacterController, KinematicCharacterControllerOutput,
    MassModifiedEvent, PhysicsWorld, RapierWorld, Real, RigidBodyDisabled, SoftCcd, WorldId,
    DEFAULT_WORLD_ID,
};
use crate::utils;
use bevy::ecs::system::{StaticSystemParam, SystemParamItem};
use bevy::prelude::*;
use rapier::prelude::*;
use std::collections::HashMap;

use super::{find_item_and_world, get_world};

#[cfg(all(feature = "dim3", feature = "async-collider"))]
use {
    crate::prelude::{AsyncCollider, AsyncSceneCollider},
    bevy::scene::SceneInstance,
};

/// Components that will be updated after a physics step.
pub type RigidBodyWritebackComponents<'a> = (
    Entity,
    Option<&'a mut Transform>,
    Option<&'a mut TransformInterpolation>,
    Option<&'a mut Velocity>,
    Option<&'a mut Sleeping>,
    Option<&'a PhysicsWorld>,
    Option<&'a RigidBody>,
);

/// Components related to rigid-bodies.
pub type RigidBodyComponents<'a> = (
    Entity,
    &'a RigidBody,
    Option<&'a GlobalTransform>,
    Option<&'a Velocity>,
    Option<&'a AdditionalMassProperties>,
    Option<&'a ReadMassProperties>,
    Option<&'a LockedAxes>,
    Option<&'a ExternalForce>,
    Option<&'a GravityScale>,
    (Option<&'a Ccd>, Option<&'a SoftCcd>),
    Option<&'a Dominance>,
    Option<&'a Sleeping>,
    (
        Option<&'a Damping>,
        Option<&'a RigidBodyDisabled>,
        Option<&'a PhysicsWorld>,
        Option<&'a AdditionalSolverIterations>,
    ),
);

/// Components related to colliders.
pub type ColliderComponents<'a> = (
    Entity,
    &'a Collider,
    Option<&'a Sensor>,
    Option<&'a ColliderMassProperties>,
    Option<&'a ActiveEvents>,
    Option<&'a ActiveHooks>,
    Option<&'a ActiveCollisionTypes>,
    Option<&'a Friction>,
    Option<&'a Restitution>,
    Option<&'a ContactSkin>,
    Option<&'a CollisionGroups>,
    Option<&'a SolverGroups>,
    Option<&'a ContactForceEventThreshold>,
    Option<&'a ColliderDisabled>,
);

// Changes the world something is in.
// This will also change the children of that entity to reflect the new world.
fn recursively_apply_world_update(
    children_query: &Query<&Children>,
    physics_world_query: &Query<(Entity, Ref<PhysicsWorld>)>,
    context: &RapierContext,
    entity: Entity,
    commands: &mut Commands,
    new_world: WorldId,
) {
    if let Ok((_, physics_world)) = physics_world_query.get(entity) {
        if physics_world.world_id != new_world {
            // Needs to insert instead of changing a mut physics_world because
            // the Ref<PhysicsWorld> requires physics_world to not be mut.
            commands.entity(entity).insert(PhysicsWorld {
                world_id: new_world,
            });
        }

        // This currently loops through every world to find it, which
        // isn't the most efficient but gets the job done.
        if context
            .get_world(new_world)
            .map(|world| world.entity2body.contains_key(&entity))
            .unwrap_or(false)
        {
            // The value of the component did not change, no need to bubble it down or remove it from the world
            return;
        }

        // This entity will be picked up by the "init_colliders" systems and added
        // to the correct world if it is missing these components.
        commands
            .entity(entity)
            .remove::<RapierColliderHandle>()
            .remove::<RapierRigidBodyHandle>()
            .remove::<RapierMultibodyJointHandle>()
            .remove::<RapierImpulseJointHandle>();
    } else {
        commands
            .entity(entity)
            .insert(PhysicsWorld {
                world_id: new_world,
            })
            .remove::<RapierColliderHandle>()
            .remove::<RapierRigidBodyHandle>()
            .remove::<RapierMultibodyJointHandle>()
            .remove::<RapierImpulseJointHandle>();
    }

    // Carries down world changes to children
    if let Ok(children) = children_query.get(entity) {
        for child in children.iter() {
            recursively_apply_world_update(
                children_query,
                physics_world_query,
                context,
                *child,
                commands,
                new_world,
            );
        }
    }
}

/// Whenever an entity has its PhysicsWorld component changed, this
/// system places it in the new world & removes it from the old.
///
/// This does NOT add the entity to the new world, only signals that
/// it needs changed. Later down the line, systems will pick up this
/// entity that needs added & do everything necessary to add it.
///
/// This system will carry this change down to the children of that entity.
pub fn apply_changing_worlds(
    mut commands: Commands,
    physics_world_query: Query<(Entity, Ref<PhysicsWorld>)>,
    children_query: Query<&Children>,
    context: Res<RapierContext>,
) {
    for (entity, physics_world) in physics_world_query.iter() {
        if physics_world.is_added() || physics_world.is_changed() {
            recursively_apply_world_update(
                &children_query,
                &physics_world_query,
                &context,
                entity,
                &mut commands,
                physics_world.world_id,
            );
        }
    }
}

/// System responsible for advancing the physics simulation, and updating the internal state
/// for scene queries.
#[allow(clippy::too_many_arguments)]
pub fn step_simulation<Hooks>(
    mut context: ResMut<RapierContext>,
    config: Res<RapierConfiguration>,
    hooks: StaticSystemParam<Hooks>,
    time: Res<Time>,
    mut sim_to_render_time: ResMut<SimulationToRenderTime>,
    mut collision_event_writer: EventWriter<CollisionEvent>,
    mut contact_force_event_writer: EventWriter<ContactForceEvent>,
    mut interpolation_query: Query<(&RapierRigidBodyHandle, &mut TransformInterpolation)>,
) where
    Hooks: 'static + BevyPhysicsHooks,
    for<'w, 's> SystemParamItem<'w, 's, Hooks>: BevyPhysicsHooks,
{
    let hooks_adapter = BevyPhysicsHooksAdapter::new(hooks.into_inner());

    for (_, world) in context.worlds.iter_mut() {
        if config.physics_pipeline_active {
            world.step_simulation(
                config.gravity,
                config.timestep_mode,
                true,
                &hooks_adapter,
                &time,
                &mut sim_to_render_time,
                &mut Some(&mut interpolation_query),
            );

            world.deleted_colliders.clear();

            world.send_bevy_events(&mut collision_event_writer, &mut contact_force_event_writer);
        } else {
            world.propagate_modified_body_positions_to_colliders();
        }

        if config.query_pipeline_active {
            world.update_query_pipeline();
        }
    }
}
