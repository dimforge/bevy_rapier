use crate::physics::{
    ColliderHandleComponent, EventQueue, JointBuilderComponent, JointHandleComponent,
    RapierConfiguration, RigidBodyHandleComponent, SimulationToRenderTime,
};

use crate::rapier::pipeline::QueryPipeline;
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
    (time, mut sim_to_render_time): (Res<Time>, ResMut<SimulationToRenderTime>),
    configuration: Res<RapierConfiguration>,
    integration_parameters: Res<IntegrationParameters>,
    (mut pipeline, mut query_pipeline): (ResMut<PhysicsPipeline>, ResMut<QueryPipeline>),
    (mut broad_phase, mut narrow_phase): (ResMut<BroadPhase>, ResMut<NarrowPhase>),
    mut bodies: ResMut<RigidBodySet>,
    mut colliders: ResMut<ColliderSet>,
    mut joints: ResMut<JointSet>,
    events: Res<EventQueue>,
) {
    if events.auto_clear {
        events.clear();
    }

    sim_to_render_time.diff += time.delta_seconds;

    let sim_dt = integration_parameters.dt();
    while sim_to_render_time.diff - sim_dt > 0.0 {
        if configuration.physics_pipeline_active {
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
        sim_to_render_time.diff -= sim_dt;
    }

    if configuration.query_pipeline_active {
        query_pipeline.update(&mut bodies, &colliders);
    }
}

/// System responsible for writing the rigid-bodies positions into the Bevy translation and rotation components.
pub fn sync_transform_system(
    sim_to_render_time: Res<SimulationToRenderTime>,
    bodies: ResMut<RigidBodySet>,
    configuration: Res<RapierConfiguration>,
    rigid_body: &RigidBodyHandleComponent,
    mut transform: Mut<Transform>,
) {
    let dt = sim_to_render_time.diff;
    if let Some(rb) = bodies.get(rigid_body.handle()) {
        #[cfg(feature = "dim2")]
        {
            // Predict position at render time
            let pos_0 = Vec2::new(
                rb.position.translation.vector.x,
                rb.position.translation.vector.y,
            );
            let vel_0 = Vec2::new(rb.linvel.x, rb.linvel.y);
            let pos_t = pos_0 + vel_0 * dt;

            // Predict rotation at render time
            let rot_0 = Quat::from_axis_angle(Vec3::unit_z(), rb.position.rotation.angle());
            let drot = Quat::from_rotation_z(rb.angvel.z * dt);
            let rot_t = drot * rot_0;

            // Do not touch the 'z' part of the translation, used in Bevy for 2d layering
            *transform.translation_mut().x_mut() = pos_t.x() * configuration.scale;
            *transform.translation_mut().y_mut() = pos_t.y() * configuration.scale;
            transform.set_rotation(rot_t);
        }

        #[cfg(feature = "dim3")]
        {
            // Predict position at render time
            let pos_0 = Vec3::new(
                rb.position.translation.vector.x,
                rb.position.translation.vector.y,
                rb.position.translation.vector.z,
            );
            let vel_0 = Vec3::new(rb.linvel.x, rb.linvel.y, rb.linvel.z);
            let pos_t = pos_0 + vel_0 * dt;

            transform.set_translation(pos_t * configuration.scale);

            // Predict rotation at render time
            if let Some(axis_na) = rb.position.rotation.axis() {
                let axis = Vec3::new(axis_na.x, axis_na.y, axis_na.z);
                let rot_0 = Quat::from_axis_angle(axis, rb.position.rotation.angle());
                let drot = Quat::from_rotation_ypr(
                    rb.angvel.y * dt,
                    rb.angvel.x * dt,
                    rb.angvel.z * dt,
                );
                let rot_t = drot * rot_0;

                transform.set_rotation(rot_t);
            }
        }
    }
}
