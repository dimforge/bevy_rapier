//! Systems responsible for interfacing our Bevy components with the Rapier physics engine.

use crate::dynamics::{
    RapierImpulseJointHandle, RapierMultibodyJointHandle, RapierRigidBodyHandle,
};
use crate::geometry::RapierColliderHandle;
use crate::plugin::RapierContext;
use crate::prelude::{PhysicsWorld, WorldId};
use bevy::prelude::*;

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
