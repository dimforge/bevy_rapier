use crate::physics::{
    ColliderHandleComponent, EventQueue, JointBuilderComponent, JointHandleComponent,
    PhysicsInterpolationComponent, RapierConfiguration, RigidBodyHandleComponent,
    SimulationToRenderTime,
};

use crate::rapier::pipeline::QueryPipeline;
use bevy::ecs::Mut;
use bevy::prelude::*;
use rapier::dynamics::{IntegrationParameters, JointSet, RigidBodyBuilder, RigidBodySet};
use rapier::geometry::{BroadPhase, ColliderBuilder, ColliderSet, NarrowPhase};
use rapier::math::Isometry;
use rapier::ncollide::utils::IsometryOps;
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
    mut query: Query<(
        &RigidBodyHandleComponent,
        &mut PhysicsInterpolationComponent,
    )>,
) {
    if events.auto_clear {
        events.clear();
    }

    sim_to_render_time.diff += time.delta_seconds;

    let sim_dt = integration_parameters.dt();
    while sim_to_render_time.diff >= sim_dt {
        if configuration.physics_pipeline_active {
            // NOTE: in this comparison we do the same computations we
            // will do for the next `while` iteration test, to make sure we
            // don't get bit by potential float inaccuracy.
            if sim_to_render_time.diff - sim_dt < sim_dt {
                // This is the last simulation step to be executed in the loop
                // Update the previous state transforms
                for (body_handle, mut previous_state) in &mut query.iter() {
                    if let Some(body) = bodies.get(body_handle.handle()) {
                        previous_state.0 = body.position;
                    }
                }
            }
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

fn sync_transform_2d(pos: Isometry<f32>, scale: f32, transform: &mut Mut<Transform>) {
    // Do not touch the 'z' part of the translation, used in Bevy for 2d layering
    *transform.translation_mut().x_mut() = pos.translation.vector.x * scale;
    *transform.translation_mut().y_mut() = pos.translation.vector.y * scale;

    let rot = na::UnitQuaternion::new(na::Vector3::z() * pos.rotation.angle());
    transform.set_rotation(Quat::from_xyzw(rot.i, rot.j, rot.k, rot.w));
}

fn sync_transform_3d(pos: Isometry<f32>, scale: f32, transform: &mut Mut<Transform>) {
    transform.set_translation(
        Vec3::new(
            pos.translation.vector.x,
            pos.translation.vector.y,
            pos.translation.vector.z,
        ) * scale,
    );
    transform.set_rotation(Quat::from_xyzw(
        pos.rotation.i,
        pos.rotation.j,
        pos.rotation.k,
        pos.rotation.w,
    ));
}

/// System responsible for writing the rigid-bodies positions into the Bevy translation and rotation components.
pub fn sync_transform_system(
    sim_to_render_time: Res<SimulationToRenderTime>,
    bodies: ResMut<RigidBodySet>,
    configuration: Res<RapierConfiguration>,
    integration_parameters: Res<IntegrationParameters>,
    mut interpolation_query: Query<(
        &RigidBodyHandleComponent,
        &PhysicsInterpolationComponent,
        &mut Transform,
    )>,
    mut direct_query: Query<
        Without<PhysicsInterpolationComponent, (&RigidBodyHandleComponent, &mut Transform)>,
    >,
) {
    let dt = sim_to_render_time.diff;
    let sim_dt = integration_parameters.dt();
    let alpha = dt / sim_dt;
    for (rigid_body, previous_pos, mut transform) in &mut interpolation_query.iter() {
        if let Some(rb) = bodies.get(rigid_body.handle()) {
            // Predict position and orientation at render time
            let pos = previous_pos.0.lerp_slerp(&rb.position, alpha);
            #[cfg(feature = "dim2")]
            sync_transform_2d(pos, configuration.scale, &mut transform);

            #[cfg(feature = "dim3")]
            sync_transform_3d(pos, configuration.scale, &mut transform);
        }
    }
    for (rigid_body, mut transform) in &mut direct_query.iter() {
        if let Some(rb) = bodies.get(rigid_body.handle()) {
            let pos = rb.position;
            #[cfg(feature = "dim2")]
            sync_transform_2d(pos, configuration.scale, &mut transform);

            #[cfg(feature = "dim3")]
            sync_transform_3d(pos, configuration.scale, &mut transform);
        }
    }
}
