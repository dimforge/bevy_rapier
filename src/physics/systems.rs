use crate::physics::{
    ColliderComponentsQuery, ColliderComponentsSet, ColliderPositionSync, ComponentSetQueryMut,
    EventQueue, IntoEntity, IntoHandle, JointBuilderComponent, JointHandleComponent,
    JointsEntityMap, ModificationTracker, PhysicsHooksWithQueryInstance,
    PhysicsHooksWithQueryObject, QueryComponentSetMut, RapierConfiguration,
    RigidBodyComponentsQuery, RigidBodyComponentsSet, RigidBodyPositionSync,
    SimulationToRenderTime, TimestepMode,
};

use crate::prelude::{ContactEvent, IntersectionEvent};
use crate::rapier::data::ComponentSetOption;
use crate::rapier::dynamics::{
    RigidBodyCcd, RigidBodyChanges, RigidBodyColliders, RigidBodyIds, RigidBodyMassProps,
    RigidBodyPosition,
};
use crate::rapier::geometry::{
    ColliderBroadPhaseData, ColliderChanges, ColliderMassProps, ColliderParent, ColliderPosition,
    ColliderShape,
};
use crate::rapier::pipeline::QueryPipeline;
use bevy::ecs::query::WorldQuery;
use bevy::prelude::*;
use rapier::dynamics::{CCDSolver, IntegrationParameters, IslandManager, JointSet};
use rapier::geometry::{BroadPhase, NarrowPhase};
use rapier::math::Isometry;
use rapier::pipeline::PhysicsPipeline;
use std::sync::RwLock;

pub const ATTACH_BODIES_AND_COLLIDERS_SYSTEM: &str = "attach_bodies_and_colliders_system";
pub const FINALIZE_COLLIDER_ATTACH_TO_BODIES_SYSTEM: &str = "finalize_collider_attach_to_bodies";
pub const CREATE_JOINTS_SYSTEM: &str = "create_joints_system";
pub const STEP_WORLD_SYSTEM: &str = "step_world_system";
pub const SYNC_TRANSFORMS_SYSTEM: &str = "sync_transforms";
pub const COLLECT_REMOVALS_SYSTEM: &str = "collect_removals";

/// System responsible for creating a Rapier rigid-body and collider from their
/// builder resources.
pub fn attach_bodies_and_colliders_system(
    mut commands: Commands,
    mut body_query: Query<(
        // Rigid-bodies.
        &RigidBodyPosition,
    )>,
    mut colliders_query: Query<
        (
            Entity,
            Option<&Parent>,
            // Colliders.
            &ColliderPosition,
        ),
        Without<ColliderParent>,
    >,
) {
    for (collider_entity, parent_entity, co_pos) in colliders_query.iter_mut() {
        let body_entity;

        if body_query.get_mut(collider_entity).is_ok() {
            // The body and collider are attached to the same entity.
            body_entity = collider_entity;
        } else if let Some(parent_entity) = parent_entity {
            body_entity = parent_entity.0;
        } else {
            continue;
        }

        let co_parent = ColliderParent {
            pos_wrt_parent: co_pos.0,
            handle: body_entity.handle(),
        };
        commands.entity(collider_entity).insert(co_parent);
    }
}

/// System responsible for creating a Rapier rigid-body and collider from their
/// builder resources.
pub fn finalize_collider_attach_to_bodies(
    mut modif_tracker: ResMut<ModificationTracker>,
    mut body_query: Query<(
        // Rigid-bodies.
        &mut RigidBodyChanges,
        &mut RigidBodyCcd,
        &mut RigidBodyColliders,
        &mut RigidBodyMassProps,
        &RigidBodyPosition,
    )>,
    mut colliders_query: Query<
        (
            Entity,
            // Collider.
            &mut ColliderChanges,
            &mut ColliderBroadPhaseData,
            &mut ColliderPosition,
            &ColliderShape,
            &ColliderMassProps,
            &ColliderParent,
        ),
        Added<ColliderParent>,
    >,
) {
    for (
        collider_entity,
        mut co_changes,
        mut co_bf_data,
        mut co_pos,
        co_shape,
        co_mprops,
        co_parent,
    ) in colliders_query.iter_mut()
    {
        if let Ok((mut rb_changes, mut rb_ccd, mut rb_colliders, mut rb_mprops, rb_pos)) =
            body_query.get_mut(co_parent.handle.entity())
        {
            // Contract:
            // - Reset collider's references.
            // - Set collider's parent handle.
            // - Attach the collider to the body.

            // Update the modification tracker.
            // NOTE: this must be done before the `.attach_collider` because
            //       `.attach_collider` will set the `MODIFIED` flag.
            if !rb_changes.contains(RigidBodyChanges::MODIFIED) {
                modif_tracker.modified_bodies.push(co_parent.handle);
            }

            modif_tracker
                .body_colliders
                .entry(co_parent.handle)
                .or_insert(vec![])
                .push(collider_entity.handle());
            modif_tracker
                .colliders_parent
                .insert(collider_entity.handle(), co_parent.handle);

            *co_changes = ColliderChanges::default();
            *co_bf_data = ColliderBroadPhaseData::default();
            rb_colliders.attach_collider(
                &mut rb_changes,
                &mut rb_ccd,
                &mut rb_mprops,
                &rb_pos,
                collider_entity.handle(),
                &mut co_pos,
                &co_parent,
                &co_shape,
                &co_mprops,
            );
        }
    }
}

/// System responsible for creating Rapier joints from their builder resources.
pub fn create_joints_system(
    mut commands: Commands,
    mut joints: ResMut<JointSet>,
    mut joints_entity_map: ResMut<JointsEntityMap>,
    query: Query<(Entity, &JointBuilderComponent)>,
    bodies: ComponentSetQueryMut<RigidBodyIds>,
) {
    let mut bodies = QueryComponentSetMut(bodies);

    for (entity, joint) in &mut query.iter() {
        // Make sure the rigid-bodies the joint it attached to exist.
        if bodies
            .0
            .q0()
            .get_component::<RigidBodyIds>(joint.entity1)
            .is_err()
            || bodies
                .0
                .q0()
                .get_component::<RigidBodyIds>(joint.entity2)
                .is_err()
        {
            continue;
        }

        let handle = joints.insert(
            &mut bodies,
            joint.entity1.handle(),
            joint.entity2.handle(),
            joint.params,
        );
        commands
            .entity(entity)
            .insert(JointHandleComponent::new(
                handle,
                joint.entity1,
                joint.entity2,
            ))
            .remove::<JointBuilderComponent>();
        joints_entity_map.0.insert(entity, handle);
    }
}

/// System responsible for performing one timestep of the physics world.
pub fn step_world_system<UserData: 'static + WorldQuery>(
    mut commands: Commands,
    (time, mut sim_to_render_time): (Res<Time>, ResMut<SimulationToRenderTime>),
    (configuration, integration_parameters): (Res<RapierConfiguration>, Res<IntegrationParameters>),
    mut modifs_tracker: ResMut<ModificationTracker>,
    (
        mut pipeline,
        mut query_pipeline,
        mut islands,
        mut broad_phase,
        mut narrow_phase,
        mut ccd_solver,
        mut joints,
        mut joints_entity_map,
    ): (
        ResMut<PhysicsPipeline>,
        ResMut<QueryPipeline>,
        ResMut<IslandManager>,
        ResMut<BroadPhase>,
        ResMut<NarrowPhase>,
        ResMut<CCDSolver>,
        ResMut<JointSet>,
        ResMut<JointsEntityMap>,
    ),
    hooks: Res<PhysicsHooksWithQueryObject<UserData>>,
    (intersection_events, contact_events): (
        EventWriter<IntersectionEvent>,
        EventWriter<ContactEvent>,
    ),
    user_data: Query<UserData>,
    mut position_sync_query: Query<(Entity, &mut RigidBodyPositionSync)>,
    bodies_query: RigidBodyComponentsQuery,
    colliders_query: ColliderComponentsQuery,
    (removed_bodies, removed_colliders, removed_joints): (
        RemovedComponents<RigidBodyChanges>,
        RemovedComponents<ColliderChanges>,
        RemovedComponents<JointHandleComponent>,
    ),
) {
    use std::mem::replace;

    let events = EventQueue {
        intersection_events: RwLock::new(intersection_events),
        contact_events: RwLock::new(contact_events),
    };
    let mut rigid_body_components_set = RigidBodyComponentsSet(bodies_query);
    let mut collider_components_set = ColliderComponentsSet(colliders_query);

    modifs_tracker.detect_removals(removed_bodies, removed_colliders, removed_joints);
    modifs_tracker.detect_modifications(
        &mut rigid_body_components_set.0,
        &mut collider_components_set.0,
    );
    modifs_tracker.propagate_removals(
        &mut commands,
        &mut islands,
        &mut rigid_body_components_set,
        &mut joints,
        &mut joints_entity_map,
    );
    islands.cleanup_removed_rigid_bodies(&mut rigid_body_components_set);

    let physics_hooks = PhysicsHooksWithQueryInstance {
        user_data,
        hooks: &*hooks.0,
    };

    match configuration.timestep_mode {
        TimestepMode::InterpolatedTimestep => {
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
                        for (entity, mut position_sync) in position_sync_query.iter_mut() {
                            if let RigidBodyPositionSync::Interpolated { prev_pos } =
                                &mut *position_sync
                            {
                                let rb_pos: Option<&RigidBodyPosition> =
                                    rigid_body_components_set.get(entity.handle());
                                if let Some(rb_pos) = rb_pos {
                                    *prev_pos = Some(rb_pos.position);
                                }
                            }
                        }
                    }

                    pipeline.step_generic(
                        &configuration.gravity,
                        &integration_parameters,
                        &mut islands,
                        &mut broad_phase,
                        &mut narrow_phase,
                        &mut rigid_body_components_set,
                        &mut collider_components_set,
                        &mut replace(&mut modifs_tracker.modified_bodies, vec![]),
                        &mut replace(&mut modifs_tracker.modified_colliders, vec![]),
                        &mut replace(&mut modifs_tracker.removed_colliders, vec![]),
                        &mut joints,
                        &mut ccd_solver,
                        &physics_hooks,
                        &events,
                    );

                    modifs_tracker.clear_modified_and_removed();
                }
                sim_to_render_time.diff -= sim_dt;
            }
        }
        TimestepMode::VariableTimestep | TimestepMode::FixedTimestep => {
            if configuration.physics_pipeline_active {
                let mut new_integration_parameters = *integration_parameters;

                if configuration.timestep_mode == TimestepMode::VariableTimestep {
                    new_integration_parameters.dt =
                        time.delta_seconds().min(integration_parameters.dt);
                }

                pipeline.step_generic(
                    &configuration.gravity,
                    &new_integration_parameters,
                    &mut islands,
                    &mut broad_phase,
                    &mut narrow_phase,
                    &mut rigid_body_components_set,
                    &mut collider_components_set,
                    &mut replace(&mut modifs_tracker.modified_bodies, vec![]),
                    &mut replace(&mut modifs_tracker.modified_colliders, vec![]),
                    &mut replace(&mut modifs_tracker.removed_colliders, vec![]),
                    &mut joints,
                    &mut ccd_solver,
                    &physics_hooks,
                    &events,
                );

                modifs_tracker.clear_modified_and_removed();
            }
        }
    }

    if configuration.query_pipeline_active {
        query_pipeline.update_generic(
            &islands,
            &mut rigid_body_components_set,
            &collider_components_set,
        );
    }
}

#[cfg(feature = "dim2")]
pub(crate) fn sync_transform(pos: &Isometry<f32>, scale: f32, transform: &mut Transform) {
    let (tra, rot) = (*pos).into();
    // Do not touch the 'z' part of the translation, used in Bevy for 2d layering
    transform.translation.x = tra.x * scale;
    transform.translation.y = tra.y * scale;
    transform.rotation = rot;
}

#[cfg(feature = "dim3")]
pub(crate) fn sync_transform(pos: &Isometry<f32>, scale: f32, transform: &mut Transform) {
    let (tra, rot) = (*pos).into();
    transform.translation = tra * scale;
    transform.rotation = rot;
}

/// System responsible for writing the rigid-bodies positions into the Bevy translation and rotation components.
pub fn sync_transforms(
    mut commands: Commands,
    sim_to_render_time: Res<SimulationToRenderTime>,
    configuration: Res<RapierConfiguration>,
    integration_parameters: Res<IntegrationParameters>,
    rigid_body_sync_mode: Query<&RigidBodyPositionSync>,
    // TODO: add some Changed filters to only sync when something moved?
    mut sync_query: QuerySet<(
        Query<(
            Entity,
            &RigidBodyPosition,
            &RigidBodyPositionSync,
            Option<&mut Transform>,
            Option<&mut GlobalTransform>,
        )>,
        Query<(
            Entity,
            &ColliderPosition,
            &ColliderPositionSync,
            Option<&ColliderParent>,
            Option<&mut Transform>,
            Option<&mut GlobalTransform>,
        )>,
    )>,
) {
    let dt = sim_to_render_time.diff;
    let sim_dt = integration_parameters.dt;
    let alpha = dt / sim_dt;

    // Sync bodies.
    for (entity, rb_pos, sync_mode, mut transform, global_transform) in
        sync_query.q0_mut().iter_mut()
    {
        let mut new_transform = transform
            .as_deref_mut()
            .map(|t| t.clone())
            .unwrap_or(Transform::identity());

        match sync_mode {
            RigidBodyPositionSync::Discrete => {
                sync_transform(&rb_pos.position, configuration.scale, &mut new_transform)
            }
            RigidBodyPositionSync::Interpolated { prev_pos } => {
                // Predict position and orientation at render time
                let mut pos = rb_pos.position;

                if configuration.timestep_mode == TimestepMode::InterpolatedTimestep
                    && prev_pos.is_some()
                {
                    pos = prev_pos.unwrap().lerp_slerp(&rb_pos.position, alpha);
                }

                sync_transform(&pos, configuration.scale, &mut new_transform);
            }
        }

        if let Some(transform) = transform.as_deref_mut() {
            *transform = new_transform;
        } else {
            commands.entity(entity).insert(new_transform);
        }

        if global_transform.is_none() {
            commands.entity(entity).insert(GlobalTransform::identity());
        }
    }

    // Sync colliders.
    for (entity, co_pos, _, co_parent, mut transform, mut global_transform) in
        sync_query.q1_mut().iter_mut()
    {
        let mut new_transform = transform
            .as_deref_mut()
            .map(|t| t.clone())
            .unwrap_or(Transform::identity());

        if let Some(co_parent) = co_parent {
            if rigid_body_sync_mode.get(co_parent.handle.entity()).is_ok() {
                // Sync the relative position instead of the actual collider position.
                sync_transform(
                    &co_parent.pos_wrt_parent,
                    configuration.scale,
                    &mut new_transform,
                );

                if let Some(transform) = transform.as_deref_mut() {
                    *transform = new_transform;
                } else {
                    commands.entity(entity).insert(new_transform);
                }

                continue;
            }
        }

        // Otherwise, sync the global position of the collider.
        sync_transform(&co_pos, configuration.scale, &mut new_transform);

        if let Some(transform) = transform.as_deref_mut() {
            *transform = new_transform;
        } else {
            commands.entity(entity).insert(new_transform);
        }

        // Just in case the parent doesn't have a Transform component
        // resulting in the global transform not being updated.
        if let Some(global_transform) = global_transform.as_mut() {
            global_transform.translation = new_transform.translation;
            global_transform.rotation = new_transform.rotation;
            global_transform.scale = new_transform.scale;
        } else {
            commands.entity(entity).insert(GlobalTransform {
                translation: new_transform.translation,
                rotation: new_transform.rotation,
                scale: new_transform.scale,
            });
        }
    }
}

/// System responsible for collecting the entities with removed rigid-bodies, colliders,
/// or joints.
pub fn collect_removals(
    mut modification_tracker: ResMut<ModificationTracker>,
    removed_bodies: RemovedComponents<RigidBodyChanges>,
    removed_colliders: RemovedComponents<ColliderChanges>,
    removed_joints: RemovedComponents<JointHandleComponent>,
) {
    modification_tracker.detect_removals(removed_bodies, removed_colliders, removed_joints);
}
