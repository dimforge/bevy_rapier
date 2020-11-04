use crate::physics::{
    ColliderHandleComponent, EntityMaps, EventQueue, InteractionPairFilters, JointBuilderComponent,
    JointHandleComponent, PhysicsInterpolationComponent, RapierConfiguration,
    RigidBodyHandleComponent, SimulationToRenderTime,
};

use crate::rapier::pipeline::QueryPipeline;
use bevy::ecs::Mut;
use bevy::prelude::*;
use rapier::dynamics::{IntegrationParameters, JointSet, RigidBodyBuilder, RigidBodySet};
use rapier::geometry::{BroadPhase, ColliderBuilder, ColliderSet, NarrowPhase};
use rapier::math::Isometry;
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
    mut entity_maps: ResMut<EntityMaps>,
    entity: Entity,
    body_builder: &RigidBodyBuilder,
    collider_builder: &ColliderBuilder,
) {
    let handle = bodies.insert(body_builder.build());
    commands.insert_one(entity, RigidBodyHandleComponent::from(handle));
    commands.remove_one::<RigidBodyBuilder>(entity);
    entity_maps.bodies.insert(entity, handle);

    let handle = colliders.insert(collider_builder.build(), handle, &mut bodies);
    commands.insert_one(entity, ColliderHandleComponent::from(handle));
    commands.remove_one::<ColliderBuilder>(entity);
    entity_maps.colliders.insert(entity, handle);
}

/// System responsible for creating Rapier joints from their builder resources.
pub fn create_joints_system(
    mut commands: Commands,
    mut bodies: ResMut<RigidBodySet>,
    mut joints: ResMut<JointSet>,
    mut entity_maps: ResMut<EntityMaps>,
    query: Query<(Entity, &JointBuilderComponent)>,
    query_bodyhandle: Query<&RigidBodyHandleComponent>,
) {
    for (entity, joint) in &mut query.iter() {
        let body1 = query_bodyhandle.get_component::<RigidBodyHandleComponent>(joint.entity1);
        let body2 = query_bodyhandle.get_component::<RigidBodyHandleComponent>(joint.entity2);
        if let (Ok(body1), Ok(body2)) = (body1, body2) {
            let handle = joints.insert(&mut bodies, body1.handle(), body2.handle(), joint.params);
            commands.insert_one(
                entity,
                JointHandleComponent::new(handle, joint.entity1, joint.entity2),
            );
            commands.remove_one::<JointBuilderComponent>(entity);
            entity_maps.joints.insert(entity, handle);
        }
    }
}

/// System responsible for performing one timestep of the physics world.
pub fn step_world_system(
    (time, mut sim_to_render_time): (Res<Time>, ResMut<SimulationToRenderTime>),
    (configuration, integration_parameters): (Res<RapierConfiguration>, Res<IntegrationParameters>),
    filter: Res<InteractionPairFilters>,
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
                for (body_handle, mut previous_state) in query.iter_mut() {
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
                filter.contact_filter.as_deref(),
                filter.proximity_filter.as_deref(),
                &*events,
            );
        }
        sim_to_render_time.diff -= sim_dt;
    }

    if configuration.query_pipeline_active {
        query_pipeline.update(&mut bodies, &colliders);
    }
}

#[cfg(feature = "dim2")]
fn sync_transform_2d(pos: Isometry<f32>, scale: f32, transform: &mut Mut<Transform>) {
    // Do not touch the 'z' part of the translation, used in Bevy for 2d layering
    *transform.translation.x_mut() = pos.translation.vector.x * scale;
    *transform.translation.y_mut() = pos.translation.vector.y * scale;

    let rot = na::UnitQuaternion::new(na::Vector3::z() * pos.rotation.angle());
    transform.rotation = Quat::from_xyzw(rot.i, rot.j, rot.k, rot.w);
}

#[cfg(feature = "dim3")]
fn sync_transform_3d(pos: Isometry<f32>, scale: f32, transform: &mut Mut<Transform>) {
    transform.translation = Vec3::new(
        pos.translation.vector.x,
        pos.translation.vector.y,
        pos.translation.vector.z,
    ) * scale;
    transform.rotation = Quat::from_xyzw(
        pos.rotation.i,
        pos.rotation.j,
        pos.rotation.k,
        pos.rotation.w,
    );
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
    for (rigid_body, previous_pos, mut transform) in interpolation_query.iter_mut() {
        if let Some(rb) = bodies.get(rigid_body.handle()) {
            // Predict position and orientation at render time
            let pos = previous_pos.0.lerp_slerp(&rb.position, alpha);
            #[cfg(feature = "dim2")]
            sync_transform_2d(pos, configuration.scale, &mut transform);

            #[cfg(feature = "dim3")]
            sync_transform_3d(pos, configuration.scale, &mut transform);
        }
    }
    for (rigid_body, mut transform) in direct_query.iter_mut() {
        if let Some(rb) = bodies.get(rigid_body.handle()) {
            let pos = rb.position;
            #[cfg(feature = "dim2")]
            sync_transform_2d(pos, configuration.scale, &mut transform);

            #[cfg(feature = "dim3")]
            sync_transform_3d(pos, configuration.scale, &mut transform);
        }
    }
}

/// System responsible for removing joints, colliders, and bodies that have
/// been removed from the scene
pub fn destroy_body_and_collider_system(
    mut commands: Commands,
    mut bodies: ResMut<RigidBodySet>,
    mut colliders: ResMut<ColliderSet>,
    mut joints: ResMut<JointSet>,
    mut entity_maps: ResMut<EntityMaps>,
    collider_query: Query<(Entity, &ColliderHandleComponent)>,
    joint_query: Query<(Entity, &JointHandleComponent)>,
    body_query: Query<(Entity, &RigidBodyHandleComponent)>,
) {
    // Components removed before this system
    let bodies_removed = body_query.removed::<RigidBodyHandleComponent>();
    let colliders_removed = collider_query.removed::<ColliderHandleComponent>();
    let joints_removed = joint_query.removed::<JointHandleComponent>();

    for entity in bodies_removed {
        if let Some(body_handle) = entity_maps.bodies.get(entity) {
            bodies.remove(*body_handle, &mut colliders, &mut joints);
            entity_maps.bodies.remove(entity);

            // Removing a body also removes its colliders and joints. If they were
            // not also removed then we must remove them here.
            commands.remove_one::<ColliderHandleComponent>(*entity);
            entity_maps.colliders.remove(entity);
            commands.remove_one::<JointHandleComponent>(*entity);
            entity_maps.joints.remove(entity);
        }
    }
    for entity in colliders_removed {
        if let Some(collider_handle) = entity_maps.colliders.get(entity) {
            colliders.remove(*collider_handle, &mut bodies);
            entity_maps.colliders.remove(entity);
        }
    }
    for entity in joints_removed {
        if let Some(joint_handle) = entity_maps.joints.get(entity) {
            joints.remove(*joint_handle, &mut bodies, true);
            entity_maps.joints.remove(entity);
        }
    }
}
