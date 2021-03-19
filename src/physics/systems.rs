use crate::physics::{
    ColliderHandleComponent, EntityMaps, EventQueue, InteractionPairFilters, JointBuilderComponent,
    JointHandleComponent, PhysicsInterpolationComponent, RapierConfiguration,
    RigidBodyHandleComponent, SimulationToRenderTime,
};

use crate::rapier::pipeline::QueryPipeline;
use bevy::prelude::*;
use rapier::dynamics::{IntegrationParameters, JointSet, RigidBodyBuilder, RigidBodySet};
use rapier::geometry::{BroadPhase, ColliderBuilder, ColliderSet, NarrowPhase};
use rapier::math::Isometry;
use rapier::pipeline::PhysicsPipeline;

/// System responsible for creating a Rapier rigid-body and collider from their
/// builder resources.
pub fn create_body_and_collider_system(
    commands: &mut Commands,
    mut bodies: ResMut<RigidBodySet>,
    mut colliders: ResMut<ColliderSet>,
    mut entity_maps: ResMut<EntityMaps>,
    standalone_body_query: Query<(Entity, &RigidBodyBuilder), Without<ColliderBuilder>>,
    body_and_collider_query: Query<(Entity, &RigidBodyBuilder, &ColliderBuilder)>,
    parented_collider_query: Query<(Entity, &Parent, &ColliderBuilder), Without<RigidBodyBuilder>>,
) {
    for (entity, body_builder) in standalone_body_query.iter() {
        let handle = bodies.insert(body_builder.build());
        commands.insert_one(entity, RigidBodyHandleComponent::from(handle));
        commands.remove_one::<RigidBodyBuilder>(entity);
        entity_maps.bodies.insert(entity, handle);
    }

    for (entity, body_builder, collider_builder) in body_and_collider_query.iter() {
        let handle = bodies.insert(body_builder.build());
        commands.insert_one(entity, RigidBodyHandleComponent::from(handle));
        commands.remove_one::<RigidBodyBuilder>(entity);
        entity_maps.bodies.insert(entity, handle);

        let handle = colliders.insert(collider_builder.build(), handle, &mut bodies);
        commands.insert_one(entity, ColliderHandleComponent::from(handle));
        commands.remove_one::<ColliderBuilder>(entity);
        entity_maps.colliders.insert(entity, handle);
    }

    for (entity, parent, collider_builder) in parented_collider_query.iter() {
        if let Some(body_handle) = entity_maps.bodies.get(&parent.0) {
            let handle = colliders.insert(collider_builder.build(), *body_handle, &mut bodies);
            commands.insert_one(entity, ColliderHandleComponent::from(handle));
            commands.remove_one::<ColliderBuilder>(entity);
            entity_maps.colliders.insert(entity, handle);
        } // warn here? panic here? do nothing?
    }
}

/// System responsible for replacing colliders on existing bodies when a new
/// builder is added.
///
/// NOTE: This only adds new colliders, the old collider is actually removed
/// by `destroy_body_and_collider_system`
pub fn update_collider_system(
    commands: &mut Commands,
    mut bodies: ResMut<RigidBodySet>,
    mut colliders: ResMut<ColliderSet>,
    mut entity_maps: ResMut<EntityMaps>,
    with_body_query: Query<
        (Entity, &RigidBodyHandleComponent, &ColliderBuilder),
        With<ColliderHandleComponent>,
    >,
    without_body_query: Query<
        (Entity, &Parent, &ColliderBuilder),
        (
            Without<RigidBodyHandleComponent>,
            With<ColliderHandleComponent>,
        ),
    >,
) {
    for (entity, body_handle, collider_builder) in with_body_query.iter() {
        let handle = colliders.insert(collider_builder.build(), body_handle.handle(), &mut bodies);
        commands.insert_one(entity, ColliderHandleComponent::from(handle));
        commands.remove_one::<ColliderBuilder>(entity);
        entity_maps.colliders.insert(entity, handle);
    }

    for (entity, parent, collider_builder) in without_body_query.iter() {
        if let Some(body_handle) = entity_maps.bodies.get(&parent.0) {
            let handle = colliders.insert(collider_builder.build(), *body_handle, &mut bodies);
            commands.insert_one(entity, ColliderHandleComponent::from(handle));
            commands.remove_one::<ColliderBuilder>(entity);
            entity_maps.colliders.insert(entity, handle);
        }
    }
}

#[test]
fn test_create_body_and_collider_system() {
    use bevy::ecs::Schedule;

    let mut resources = Resources::default();
    resources.insert(RigidBodySet::new());
    resources.insert(ColliderSet::new());
    resources.insert(EntityMaps::default());

    let mut world = World::new();
    let body_and_collider_entity =
        world.spawn((RigidBodyBuilder::new_dynamic(), ColliderBuilder::ball(1.0)));

    let body_only_entity = world.spawn((
        RigidBodyBuilder::new_static(),
        Parent(body_and_collider_entity),
    ));

    let parented_collider_entity_1 =
        world.spawn((Parent(body_and_collider_entity), ColliderBuilder::ball(0.5)));

    let parented_collider_entity_2 =
        world.spawn((Parent(body_only_entity), ColliderBuilder::ball(0.25)));

    let mut schedule = Schedule::default();
    schedule.add_stage("physics_test", SystemStage::parallel());
    schedule.add_system_to_stage("physics_test", create_body_and_collider_system.system());
    schedule.initialize_and_run(&mut world, &mut resources);

    let body_set = resources.get::<RigidBodySet>().unwrap();
    let collider_set = resources.get::<ColliderSet>().unwrap();
    let entity_maps = resources.get::<EntityMaps>().unwrap();

    // body attached alongside collider
    let attached_body_handle = world
        .get::<RigidBodyHandleComponent>(body_and_collider_entity)
        .unwrap()
        .handle();
    assert_eq!(
        entity_maps.bodies[&body_and_collider_entity],
        attached_body_handle
    );
    assert!(body_set.get(attached_body_handle).unwrap().is_dynamic());

    // collider attached from same entity
    let collider_handle = world
        .get::<ColliderHandleComponent>(body_and_collider_entity)
        .unwrap()
        .handle();
    assert_eq!(
        entity_maps.colliders[&body_and_collider_entity],
        collider_handle
    );
    let collider = collider_set.get(collider_handle).unwrap();
    assert_eq!(attached_body_handle, collider.parent());
    assert_eq!(collider.shape().as_ball().unwrap().radius, 1.0);

    // collider attached to child entity of body with collider
    let collider_handle = world
        .get::<ColliderHandleComponent>(parented_collider_entity_1)
        .unwrap()
        .handle();
    assert_eq!(
        entity_maps.colliders[&parented_collider_entity_1],
        collider_handle
    );
    let collider = collider_set.get(collider_handle).unwrap();
    assert_eq!(attached_body_handle, collider.parent());
    assert_eq!(collider.shape().as_ball().unwrap().radius, 0.5);

    // standalone body with no collider, jointed to the attached body
    let standalone_body_handle = world
        .get::<RigidBodyHandleComponent>(body_only_entity)
        .unwrap()
        .handle();
    assert_eq!(
        entity_maps.bodies[&body_only_entity],
        standalone_body_handle
    );
    assert!(body_set.get(standalone_body_handle).unwrap().is_static());

    // collider attached to child entity of standlone body
    let collider_handle = world
        .get::<ColliderHandleComponent>(parented_collider_entity_2)
        .unwrap()
        .handle();
    assert_eq!(
        entity_maps.colliders[&parented_collider_entity_2],
        collider_handle
    );
    let collider = collider_set.get(collider_handle).unwrap();
    assert_eq!(standalone_body_handle, collider.parent());
    assert_eq!(collider.shape().as_ball().unwrap().radius, 0.25);
}

/// System responsible for creating Rapier joints from their builder resources.
pub fn create_joints_system(
    commands: &mut Commands,
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
    events: ResMut<EventQueue>,
    mut query: Query<(
        &RigidBodyHandleComponent,
        &mut PhysicsInterpolationComponent,
    )>,
) {
    if events.auto_clear {
        events.clear();
    }

    if configuration.time_dependent_number_of_timesteps {
        sim_to_render_time.diff += time.delta_seconds();

        let sim_dt = integration_parameters.dt;
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
                            previous_state.0 = Some(*body.position());
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
                    filter.intersection_filter.as_deref(),
                    &*events,
                );
            }
            sim_to_render_time.diff -= sim_dt;
        }
    } else if configuration.physics_pipeline_active {
        pipeline.step(
            &configuration.gravity,
            &integration_parameters,
            &mut broad_phase,
            &mut narrow_phase,
            &mut bodies,
            &mut colliders,
            &mut joints,
            filter.contact_filter.as_deref(),
            filter.intersection_filter.as_deref(),
            &*events,
        );
    }

    if configuration.query_pipeline_active {
        query_pipeline.update(&mut bodies, &colliders);
    }
}

#[cfg(feature = "dim2")]
pub(crate) fn sync_transform(pos: &Isometry<f32>, scale: f32, transform: &mut Transform) {
    // Do not touch the 'z' part of the translation, used in Bevy for 2d layering
    transform.translation.x = pos.translation.vector.x * scale;
    transform.translation.y = pos.translation.vector.y * scale;

    let rot = na::UnitQuaternion::new(na::Vector3::z() * pos.rotation.angle());
    transform.rotation = Quat::from_xyzw(rot.i, rot.j, rot.k, rot.w);
}

#[cfg(feature = "dim3")]
pub(crate) fn sync_transform(pos: &Isometry<f32>, scale: f32, transform: &mut Transform) {
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
        (&RigidBodyHandleComponent, &mut Transform),
        Without<PhysicsInterpolationComponent>,
    >,
) {
    let dt = sim_to_render_time.diff;
    let sim_dt = integration_parameters.dt;
    let alpha = dt / sim_dt;
    for (rigid_body, previous_pos, mut transform) in interpolation_query.iter_mut() {
        if let Some(rb) = bodies.get(rigid_body.handle()) {
            // Predict position and orientation at render time
            let mut pos = *rb.position();

            if configuration.time_dependent_number_of_timesteps && previous_pos.0.is_some() {
                pos = previous_pos.0.unwrap().lerp_slerp(rb.position(), alpha);
            }

            sync_transform(&pos, configuration.scale, &mut transform);
        }
    }
    for (rigid_body, mut transform) in direct_query.iter_mut() {
        if let Some(rb) = bodies.get(rigid_body.handle()) {
            sync_transform(rb.position(), configuration.scale, &mut transform);
        }
    }
}

/// System responsible for removing joints, colliders, and bodies that have
/// been removed from the scene
pub fn destroy_body_and_collider_system(
    commands: &mut Commands,
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
            colliders.remove(*collider_handle, &mut bodies, true);
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
