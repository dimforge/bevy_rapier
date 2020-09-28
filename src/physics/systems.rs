use crate::physics::{
    ColliderHandleComponent, EventQueue, JointBuilderComponent,
    JointHandleComponent, RapierConfiguration, RigidBodyHandleComponent,
};

use bevy::ecs::Mut;
use bevy::prelude::*;
use rapier::dynamics::{IntegrationParameters, JointSet, RigidBodyBuilder, RigidBodySet};
use rapier::geometry::{BroadPhase, ColliderBuilder, ColliderSet, NarrowPhase};
use rapier::pipeline::PhysicsPipeline;

// TODO: right now we only support one collider attached to one body.
// This should be extanded to multiple bodies.
// The reason why we build the body and the collider in the same system is
// because systems run in parallel. This implies that if the collider creation
// system runs before the body creation system, then it won't be able to create
// the colliders because the related rigid-body handles don't exist yet. This
// causes things to be initialized during multiple frames instead of just one.
/// System responsible for creating a Rapier rigid-body and collider from their
/// builder resources.
pub fn create_body_and_collider_system(
    mut commands: Commands,
    mut bodies: ResMut<RigidBodySet>,
    mut colliders: ResMut<ColliderSet>,
    entity: Entity,
    body_builder: &RigidBodyBuilder,
    collider_builder: &ColliderBuilder,
) {
    let handle = bodies.insert(body_builder.build());
    commands.insert_one(entity, RigidBodyHandleComponent::from(handle));
    commands.remove_one::<RigidBodyBuilder>(entity);

    let handle = colliders.insert(collider_builder.build(), handle, &mut bodies);
    commands.insert_one(entity, ColliderHandleComponent::from(handle));
    commands.remove_one::<ColliderBuilder>(entity);
}

/// System responsible for creating Rapier joints from their builder resources.
pub fn create_joints_system(
    mut commands: Commands,
    mut bodies: ResMut<RigidBodySet>,
    mut joints: ResMut<JointSet>,
    mut query: Query<(Entity, &JointBuilderComponent)>,
    query_bodyhandle: Query<&RigidBodyHandleComponent>,
) {
    for (entity, joint) in &mut query.iter() {
        let body1 = query_bodyhandle.get::<RigidBodyHandleComponent>(joint.entity1);
        let body2 = query_bodyhandle.get::<RigidBodyHandleComponent>(joint.entity2);
        if let (Ok(body1), Ok(body2)) = (body1, body2) {
            let handle = joints.insert(&mut bodies, body1.handle(), body2.handle(), joint.params);
            commands.insert_one(
                entity,
                JointHandleComponent::new(handle, joint.entity1, joint.entity2),
            );
            commands.remove_one::<JointBuilderComponent>(entity);
        }
    }
}

/// System responsible for performing one timestep of the physics world.
pub fn step_world_system(
    configuration: Res<RapierConfiguration>,
    integration_parameters: Res<IntegrationParameters>,
    mut pipeline: ResMut<PhysicsPipeline>,
    mut broad_phase: ResMut<BroadPhase>,
    mut narrow_phase: ResMut<NarrowPhase>,
    mut bodies: ResMut<RigidBodySet>,
    mut colliders: ResMut<ColliderSet>,
    mut joints: ResMut<JointSet>,
    events: Res<EventQueue>,
) {
    if events.auto_clear {
        events.clear();
    }

    if configuration.active {
        pipeline.step(
            &configuration.gravity,
            &integration_parameters,
            &mut broad_phase,
            &mut narrow_phase,
            &mut bodies,
            &mut colliders,
            &mut joints,
            &*events,
        );
    }
}

/// System responsible for writing the rigid-bodies positions into the Bevy translation and rotation components.
pub fn sync_transform_system(
    bodies: ResMut<RigidBodySet>,
    configuration: Res<RapierConfiguration>,
    rigid_body: &RigidBodyHandleComponent,
    mut transform: Mut<Transform>,
) {
    if let Some(rb) = bodies.get(rigid_body.handle()) {
        let pos = rb.position;

        #[cfg(feature = "dim2")]
        {
            let rot = na::UnitQuaternion::new(na::Vector3::z() * pos.rotation.angle());
            // Do not touch the 'z' part of the translation, used in Bevy for 2d layering
            *transform.translation_mut().x_mut() = pos.translation.vector.x * configuration.scale;
            *transform.translation_mut().y_mut() = pos.translation.vector.y * configuration.scale;
            transform.set_rotation(Quat::from_xyzw(rot.i, rot.j, rot.k, rot.w));
        }

        #[cfg(feature = "dim3")]
        {
            transform.set_translation(
                Vec3::new(
                    pos.translation.vector.x,
                    pos.translation.vector.y,
                    pos.translation.vector.z,
                ) * configuration.scale,
            );

            transform.set_rotation(Quat::from_xyzw(
                pos.rotation.i,
                pos.rotation.j,
                pos.rotation.k,
                pos.rotation.w,
            ));
        }
    }
}
