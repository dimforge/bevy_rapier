//! Systems responsible for interfacing our Bevy components with the Rapier physics engine.

use crate::dynamics::{
    AdditionalMassProperties, Ccd, Damping, Dominance, ExternalForce, ExternalImpulse,
    GravityScale, ImpulseJoint, LockedAxes, MassProperties, MultibodyJoint,
    RapierImpulseJointHandle, RapierMultibodyJointHandle, RapierRigidBodyHandle, RigidBody,
    Sleeping, TransformInterpolation, Velocity,
};
use crate::geometry::{
    ActiveCollisionTypes, ActiveEvents, ActiveHooks, Collider, ColliderMassProperties,
    ColliderScale, CollisionGroups, Friction, RapierColliderHandle, Restitution, Sensor,
    SolverGroups,
};
use crate::pipeline::{
    CollisionEvent, PhysicsHooksWithQueryInstance, PhysicsHooksWithQueryResource,
};
use crate::plugin::configuration::{SimulationToRenderTime, TimestepMode};
use crate::plugin::{RapierConfiguration, RapierContext};
use crate::prelude::CollidingEntities;
use crate::utils;
use bevy::ecs::query::WorldQuery;
use bevy::math::Vec3Swizzles;
use bevy::prelude::*;
use rapier::prelude::*;
use std::collections::HashMap;

#[cfg(feature = "dim3")]
use crate::prelude::{AsyncCollider, AsyncSceneCollider};

/// Components that will be updated after a physics step.
pub type RigidBodyWritebackComponents<'a> = (
    Entity,
    Option<&'a Parent>,
    Option<&'a mut Transform>,
    Option<&'a mut TransformInterpolation>,
    Option<&'a mut Velocity>,
    Option<&'a mut Sleeping>,
);

/// Components related to rigid-bodies.
pub type RigidBodyComponents<'a> = (
    Entity,
    &'a RigidBody,
    Option<&'a GlobalTransform>,
    Option<&'a Velocity>,
    Option<&'a AdditionalMassProperties>,
    Option<&'a MassProperties>,
    Option<&'a LockedAxes>,
    Option<&'a ExternalForce>,
    Option<&'a ExternalImpulse>,
    Option<&'a GravityScale>,
    Option<&'a Ccd>,
    Option<&'a Dominance>,
    Option<&'a Sleeping>,
    Option<&'a Damping>,
);

/// Components related to colliders.
pub type ColliderComponents<'a> = (
    Entity,
    &'a Collider,
    Option<&'a GlobalTransform>,
    Option<&'a Sensor>,
    Option<&'a ColliderMassProperties>,
    Option<&'a ActiveEvents>,
    Option<&'a ActiveHooks>,
    Option<&'a ActiveCollisionTypes>,
    Option<&'a Friction>,
    Option<&'a Restitution>,
    Option<&'a CollisionGroups>,
    Option<&'a SolverGroups>,
);

/// System responsible for applying [`GlobalTransform::scale`] and/or [`ColliderScale`] to
/// colliders.
pub fn apply_scale(
    config: Res<RapierConfiguration>,
    mut changed_collider_scales: Query<
        (&mut Collider, &GlobalTransform, Option<&ColliderScale>),
        Or<(
            Changed<Collider>,
            Changed<GlobalTransform>,
            Changed<ColliderScale>,
        )>,
    >,
) {
    // NOTE: we don’t have to apply the RapierConfiguration::physics_scale here because
    //       we are applying the scale to the user-facing shape here, not the ones inside
    //       colliders (yet).
    for (mut shape, transform, custom_scale) in changed_collider_scales.iter_mut() {
        #[cfg(feature = "dim2")]
        let effective_scale = match custom_scale {
            Some(ColliderScale::Absolute(scale)) => *scale,
            Some(ColliderScale::Relative(scale)) => *scale * transform.scale.xy(),
            None => transform.scale.xy(),
        };
        #[cfg(feature = "dim3")]
        let effective_scale = match custom_scale {
            Some(ColliderScale::Absolute(scale)) => *scale,
            Some(ColliderScale::Relative(scale)) => *scale * transform.scale,
            None => transform.scale,
        };

        if shape.scale != effective_scale {
            shape.set_scale(effective_scale, config.scaled_shape_subdivision);
        }
    }
}

/// System responsible for applying changes the user made to a collider-related component.
pub fn apply_collider_user_changes(
    config: Res<RapierConfiguration>,
    mut context: ResMut<RapierContext>,
    changed_collider_transforms: Query<
        (&RapierColliderHandle, &GlobalTransform),
        (Without<RapierRigidBodyHandle>, Changed<GlobalTransform>),
    >,
    changed_shapes: Query<(&RapierColliderHandle, &Collider), Changed<Collider>>,
    changed_active_events: Query<(&RapierColliderHandle, &ActiveEvents), Changed<ActiveEvents>>,
    changed_active_hooks: Query<(&RapierColliderHandle, &ActiveHooks), Changed<ActiveHooks>>,
    changed_active_collision_types: Query<
        (&RapierColliderHandle, &ActiveCollisionTypes),
        Changed<ActiveCollisionTypes>,
    >,
    changed_friction: Query<(&RapierColliderHandle, &Friction), Changed<Friction>>,
    changed_restitution: Query<(&RapierColliderHandle, &Restitution), Changed<Restitution>>,
    changed_collision_groups: Query<
        (&RapierColliderHandle, &CollisionGroups),
        Changed<CollisionGroups>,
    >,
    changed_solver_groups: Query<(&RapierColliderHandle, &SolverGroups), Changed<SolverGroups>>,
    changed_sensors: Query<(&RapierColliderHandle, &Sensor), Changed<Sensor>>,
) {
    let scale = context.physics_scale;

    for (handle, transform) in changed_collider_transforms.iter() {
        if let Some(co) = context.colliders.get_mut(handle.0) {
            if co.parent().is_none() {
                co.set_position(utils::transform_to_iso(transform, scale))
            }
        }
    }

    for (handle, shape) in changed_shapes.iter() {
        if let Some(co) = context.colliders.get_mut(handle.0) {
            let mut scaled_shape = shape.clone();
            scaled_shape.set_scale(shape.scale / scale, config.scaled_shape_subdivision);
            co.set_shape(scaled_shape.raw.clone())
        }
    }

    for (handle, active_events) in changed_active_events.iter() {
        if let Some(co) = context.colliders.get_mut(handle.0) {
            co.set_active_events((*active_events).into())
        }
    }

    for (handle, active_hooks) in changed_active_hooks.iter() {
        if let Some(co) = context.colliders.get_mut(handle.0) {
            co.set_active_hooks((*active_hooks).into())
        }
    }

    for (handle, active_collision_types) in changed_active_collision_types.iter() {
        if let Some(co) = context.colliders.get_mut(handle.0) {
            co.set_active_collision_types((*active_collision_types).into())
        }
    }

    for (handle, friction) in changed_friction.iter() {
        if let Some(co) = context.colliders.get_mut(handle.0) {
            co.set_friction(friction.coefficient);
            co.set_friction_combine_rule(friction.combine_rule.into());
        }
    }

    for (handle, restitution) in changed_restitution.iter() {
        if let Some(co) = context.colliders.get_mut(handle.0) {
            co.set_restitution(restitution.coefficient);
            co.set_restitution_combine_rule(restitution.combine_rule.into());
        }
    }

    for (handle, collision_groups) in changed_collision_groups.iter() {
        if let Some(co) = context.colliders.get_mut(handle.0) {
            co.set_collision_groups((*collision_groups).into());
        }
    }

    for (handle, solver_groups) in changed_solver_groups.iter() {
        if let Some(co) = context.colliders.get_mut(handle.0) {
            co.set_solver_groups((*solver_groups).into());
        }
    }

    for (handle, sensor) in changed_sensors.iter() {
        if let Some(co) = context.colliders.get_mut(handle.0) {
            co.set_sensor(sensor.0)
        }
    }
}

/// System responsible for applying changes the user made to a rigid-body-related component.
pub fn apply_rigid_body_user_changes(
    mut context: ResMut<RapierContext>,
    changed_rb_types: Query<(&RapierRigidBodyHandle, &RigidBody), Changed<RigidBody>>,
    mut changed_transforms: Query<
        (
            &RapierRigidBodyHandle,
            &GlobalTransform,
            Option<&mut TransformInterpolation>,
        ),
        Changed<GlobalTransform>,
    >,
    changed_velocities: Query<(&RapierRigidBodyHandle, &Velocity), Changed<Velocity>>,
    changed_additional_mass_props: Query<
        (&RapierRigidBodyHandle, &AdditionalMassProperties),
        Changed<AdditionalMassProperties>,
    >,
    changed_locked_axes: Query<(&RapierRigidBodyHandle, &LockedAxes), Changed<LockedAxes>>,
    changed_forces: Query<(&RapierRigidBodyHandle, &ExternalForce), Changed<ExternalForce>>,
    changed_impulses: Query<(&RapierRigidBodyHandle, &ExternalImpulse), Changed<ExternalImpulse>>,
    changed_gravity_scale: Query<(&RapierRigidBodyHandle, &GravityScale), Changed<GravityScale>>,
    changed_ccd: Query<(&RapierRigidBodyHandle, &Ccd), Changed<Ccd>>,
    changed_dominance: Query<(&RapierRigidBodyHandle, &Dominance), Changed<Dominance>>,
    changed_sleeping: Query<(&RapierRigidBodyHandle, &Sleeping), Changed<Sleeping>>,
    changed_damping: Query<(&RapierRigidBodyHandle, &Damping), Changed<Damping>>,
) {
    let context = &mut *context;
    let scale = context.physics_scale;

    // Deal with sleeping first, because other changes may then wake-up the
    // rigid-body again.
    for (handle, sleeping) in changed_sleeping.iter() {
        if let Some(rb) = context.bodies.get_mut(handle.0) {
            let activation = rb.activation_mut();
            activation.linear_threshold = sleeping.linear_threshold;
            activation.angular_threshold = sleeping.angular_threshold;

            if !sleeping.sleeping && activation.sleeping {
                rb.wake_up(true);
            } else if sleeping.sleeping && !activation.sleeping {
                rb.sleep();
            }
        }
    }

    // NOTE: we must change the rigid-body type before updating the
    //       transform or velocity. Otherwise, if the rigid-body was fixed
    //       and changed to anything else, the velocity change wouldn’t have any effect.
    //       Similarly, if the rigid-body was kinematic position-based before and
    //       changed to anything else, a transform change would modify the next
    //       position instead of the current one.
    for (handle, rb_type) in changed_rb_types.iter() {
        if let Some(rb) = context.bodies.get_mut(handle.0) {
            rb.set_body_type((*rb_type).into());
        }
    }

    // Manually checks if the transform changed.
    // This is needed for detecting if the user actually changed the rigid-body
    // transform, or if it was just the change we made in our `writeback_rigid_bodies`
    // system.
    let transform_changed =
        |handle: &RigidBodyHandle,
         transform: &GlobalTransform,
         last_transform_set: &HashMap<RigidBodyHandle, GlobalTransform>| {
            if let Some(prev) = last_transform_set.get(handle) {
                let tra_changed = if cfg!(feature = "dim2") {
                    // In 2D, ignore the z component which can be changed by the user
                    // without affecting the physics.
                    transform.translation.xy() != prev.translation.xy()
                } else {
                    transform.translation != prev.translation
                };

                tra_changed || prev.rotation != transform.rotation
            } else {
                true
            }
        };

    for (handle, transform, mut interpolation) in changed_transforms.iter_mut() {
        if let Some(interpolation) = interpolation.as_deref_mut() {
            // Reset the interpolation so we don’t overwrite
            // the user’s input.
            interpolation.start = None;
            interpolation.end = None;
        }

        if let Some(rb) = context.bodies.get_mut(handle.0) {
            match rb.body_type() {
                RigidBodyType::KinematicPositionBased => {
                    if transform_changed(&handle.0, transform, &context.last_body_transform_set) {
                        rb.set_next_kinematic_position(utils::transform_to_iso(transform, scale));
                        context.last_body_transform_set.insert(handle.0, *transform);
                    }
                }
                _ => {
                    if transform_changed(&handle.0, transform, &context.last_body_transform_set) {
                        rb.set_position(utils::transform_to_iso(transform, scale), true);
                        context.last_body_transform_set.insert(handle.0, *transform);
                    }
                }
            }
        }
    }

    for (handle, velocity) in changed_velocities.iter() {
        if let Some(rb) = context.bodies.get_mut(handle.0) {
            rb.set_linvel((velocity.linvel / scale).into(), true);
            #[allow(clippy::useless_conversion)] // Need to convert if dim3 enabled
            rb.set_angvel(velocity.angvel.into(), true);
        }
    }

    for (handle, mprops) in changed_additional_mass_props.iter() {
        if let Some(rb) = context.bodies.get_mut(handle.0) {
            rb.set_additional_mass_properties(mprops.0.into_rapier(scale), true);
        }
    }

    for (handle, locked_axes) in changed_locked_axes.iter() {
        if let Some(rb) = context.bodies.get_mut(handle.0) {
            rb.set_locked_axes((*locked_axes).into(), true);
        }
    }

    for (handle, forces) in changed_forces.iter() {
        if let Some(rb) = context.bodies.get_mut(handle.0) {
            rb.reset_forces(true);
            rb.reset_torques(true);
            rb.add_force((forces.force / scale).into(), true);
            #[allow(clippy::useless_conversion)] // Need to convert if dim3 enabled
            rb.add_torque(forces.torque.into(), true);
        }
    }

    for (handle, impulses) in changed_impulses.iter() {
        if let Some(rb) = context.bodies.get_mut(handle.0) {
            rb.apply_impulse((impulses.impulse / scale).into(), true);
            #[allow(clippy::useless_conversion)] // Need to convert if dim3 enabled
            rb.apply_torque_impulse(impulses.torque_impulse.into(), true);
        }
    }

    for (handle, gravity_scale) in changed_gravity_scale.iter() {
        if let Some(rb) = context.bodies.get_mut(handle.0) {
            rb.set_gravity_scale(gravity_scale.0, true);
        }
    }

    for (handle, ccd) in changed_ccd.iter() {
        if let Some(rb) = context.bodies.get_mut(handle.0) {
            rb.enable_ccd(ccd.enabled);
        }
    }

    for (handle, dominance) in changed_dominance.iter() {
        if let Some(rb) = context.bodies.get_mut(handle.0) {
            rb.set_dominance_group(dominance.groups);
        }
    }

    for (handle, damping) in changed_damping.iter() {
        if let Some(rb) = context.bodies.get_mut(handle.0) {
            rb.set_linear_damping(damping.linear_damping);
            rb.set_angular_damping(damping.angular_damping);
        }
    }
}

/// System responsible for applying changes the user made to a joint component.
pub fn apply_joint_user_changes(
    mut context: ResMut<RapierContext>,
    changed_impulse_joints: Query<
        (&RapierImpulseJointHandle, &ImpulseJoint),
        Changed<ImpulseJoint>,
    >,
    changed_multibody_joints: Query<
        (&RapierMultibodyJointHandle, &MultibodyJoint),
        Changed<MultibodyJoint>,
    >,
) {
    let scale = context.physics_scale;

    // TODO: right now, we only support propagating changes made to the joint data.
    //       Re-parenting the joint isn’t supported yet.
    for (handle, changed_joint) in changed_impulse_joints.iter() {
        if let Some(joint) = context.impulse_joints.get_mut(handle.0) {
            joint.data = changed_joint.data.into_rapier(scale);
        }
    }

    for (handle, changed_joint) in changed_multibody_joints.iter() {
        // TODO: not sure this will always work properly, e.g., if the number of Dofs is changed.
        if let Some((mb, link_id)) = context.multibody_joints.get_mut(handle.0) {
            if let Some(link) = mb.link_mut(link_id) {
                link.joint.data = changed_joint.data.into_rapier(scale);
            }
        }
    }
}

/// System responsible for writing the result of the last simulation step into our `bevy_rapier`
/// components and the [`GlobalTransform`] component.
pub fn writeback_rigid_bodies(
    mut context: ResMut<RapierContext>,
    config: Res<RapierConfiguration>,
    sim_to_render_time: Res<SimulationToRenderTime>,
    global_transforms: Query<&GlobalTransform>,
    mut writeback: Query<RigidBodyWritebackComponents>,
) {
    let context = &mut *context;
    let scale = context.physics_scale;

    if config.physics_pipeline_active {
        for (entity, parent, transform, mut interpolation, mut velocity, mut sleeping) in
            writeback.iter_mut()
        {
            // TODO: do this the other way round: iterate through Rapier’s RigidBodySet on the active bodies,
            // and update the components accordingly. That way, we don’t have to iterate through the entities that weren’t changed
            // by physics (for example because they are sleeping).
            if let Some(handle) = context.entity2body.get(&entity).copied() {
                if let Some(rb) = context.bodies.get(handle) {
                    let mut interpolated_pos = utils::iso_to_transform(rb.position(), scale);

                    if let TimestepMode::Interpolated { dt, .. } = config.timestep_mode {
                        if let Some(interpolation) = interpolation.as_deref_mut() {
                            if interpolation.end.is_none() {
                                interpolation.end = Some(*rb.position());
                            }

                            if let Some(interpolated) =
                                interpolation.lerp_slerp((dt + sim_to_render_time.diff) / dt)
                            {
                                interpolated_pos = utils::iso_to_transform(&interpolated, scale);
                            }
                        }
                    }

                    if let Some(mut transform) = transform {
                        // NOTE: we query the parent’s global transform here, which is a bit
                        //       unfortunate (performance-wise). An alternative would be to
                        //       deduce the parent’s global transform from the current entity’s
                        //       global transform. However, this makes it nearly impossible
                        //       (because of rounding errors) to predict the exact next value this
                        //       entity’s global transform will get after the next transform
                        //       propagation, which breaks our transform modification detection
                        //       that we do to detect if the user’s transform has to be written
                        //       into the rigid-body.
                        if let Some(parent_global_transform) =
                            parent.and_then(|p| global_transforms.get(**p).ok())
                        {
                            // In 2D, preserve the transform `z` component that may have been set by the user
                            #[cfg(feature = "dim2")]
                            let prev_z = transform.translation.z;

                            // We need to compute the new local transform such that:
                            // curr_parent_global_transform * new_transform = interpolated_pos
                            // new_transform = curr_parent_global_transform.inverse() * interpolated_pos
                            let inv_parent_global_rot =
                                parent_global_transform.rotation.conjugate();
                            transform.rotation = inv_parent_global_rot * interpolated_pos.rotation;
                            transform.translation = inv_parent_global_rot
                                * (interpolated_pos.translation
                                    - parent_global_transform.translation);

                            #[cfg(feature = "dim2")]
                            {
                                transform.translation.z = prev_z;
                            }

                            // NOTE: we need to compute the result of the next transform propagation
                            //       to make sure that our change detection for transforms is exact
                            //       despite rounding errors.
                            let new_global_transform =
                                parent_global_transform.mul_transform(*transform);

                            context
                                .last_body_transform_set
                                .insert(handle, new_global_transform);
                        } else {
                            // In 2D, preserve the transform `z` component that may have been set by the user
                            #[cfg(feature = "dim2")]
                            {
                                interpolated_pos.translation.z = transform.translation.z;
                            }

                            transform.rotation = interpolated_pos.rotation;
                            transform.translation = interpolated_pos.translation;

                            context
                                .last_body_transform_set
                                .insert(handle, interpolated_pos);
                        }
                    }

                    if let Some(velocity) = &mut velocity {
                        let new_vel = Velocity {
                            linvel: (rb.linvel() * scale).into(),
                            #[cfg(feature = "dim3")]
                            angvel: (*rb.angvel()).into(),
                            #[cfg(feature = "dim2")]
                            angvel: rb.angvel(),
                        };

                        // NOTE: we write the new value only if there was an
                        //       actual change, in order to not trigger bevy’s
                        //       change tracking when the values didn’t change.
                        if **velocity != new_vel {
                            **velocity = new_vel;
                        }
                    }

                    if let Some(sleeping) = &mut sleeping {
                        // NOTE: we write the new value only if there was an
                        //       actual change, in order to not trigger bevy’s
                        //       change tracking when the values didn’t change.
                        if sleeping.sleeping != rb.is_sleeping() {
                            sleeping.sleeping = rb.is_sleeping();
                        }
                    }
                }
            }
        }
    }
}

/// System responsible for advancing the physics simulation, and updating the internal state
/// for scene queries.
pub fn step_simulation<PhysicsHooksData: 'static + WorldQuery + Send + Sync>(
    mut context: ResMut<RapierContext>,
    config: Res<RapierConfiguration>,
    hooks: Res<PhysicsHooksWithQueryResource<PhysicsHooksData>>,
    (time, mut sim_to_render_time): (Res<Time>, ResMut<SimulationToRenderTime>),
    events: EventWriter<CollisionEvent>,
    hooks_data: Query<PhysicsHooksData>,
    interpolation_query: Query<(&RapierRigidBodyHandle, &mut TransformInterpolation)>,
) {
    let context = &mut *context;

    if config.physics_pipeline_active {
        let hooks_instance = PhysicsHooksWithQueryInstance {
            user_data: hooks_data,
            hooks: &*hooks.0,
        };

        context.step_simulation(
            config.gravity,
            config.timestep_mode,
            Some(events),
            &hooks_instance,
            &*time,
            &mut *sim_to_render_time,
            Some(interpolation_query),
        );
        context.deleted_colliders.clear();
    } else {
        context.propagate_modified_body_positions_to_colliders();
    }

    if config.query_pipeline_active {
        context.update_query_pipeline();
    }
}

/// NOTE: This currently does nothing in 2D.
#[cfg(feature = "dim2")]
pub fn init_async_colliders() {}

/// System responsible for creating `Collider` components from `AsyncCollider` components if the
/// corresponding mesh has become available.
#[cfg(feature = "dim3")]
pub fn init_async_colliders(
    mut commands: Commands,
    meshes: Res<Assets<Mesh>>,
    async_colliders: Query<(Entity, &AsyncCollider)>,
) {
    for (entity, async_collider) in async_colliders.iter() {
        if let Some(mesh) = meshes.get(&async_collider.handle) {
            match Collider::from_bevy_mesh(mesh, &async_collider.shape) {
                Some(collider) => {
                    commands
                        .entity(entity)
                        .insert(collider)
                        .remove::<AsyncCollider>();
                }
                None => error!("Unable to generate collider from mesh {:?}", mesh),
            }
        }
    }
}

/// System responsible for creating `Collider` components from `AsyncSceneCollider` components if the
/// corresponding scene has become available.
#[cfg(feature = "dim3")]
pub fn init_async_scene_colliders(
    mut commands: Commands,
    meshes: Res<Assets<Mesh>>,
    scenes: Res<Assets<Scene>>,
    async_colliders: Query<(Entity, &AsyncSceneCollider)>,
    children: Query<&Children>,
    mesh_handles: Query<(&Name, &Handle<Mesh>)>,
) {
    for (entity, async_collider) in async_colliders.iter() {
        if scenes.get(&async_collider.handle).is_some() {
            traverse_descendants(entity, &children, &mut |child| {
                if let Ok((name, handle)) = mesh_handles.get(child) {
                    let shape = async_collider
                        .named_shapes
                        .get(name.as_str())
                        .unwrap_or(&async_collider.shape);
                    if let Some(shape) = shape {
                        let mesh = meshes.get(handle).unwrap(); // NOTE: Mesh is already loaded
                        match Collider::from_bevy_mesh(mesh, shape) {
                            Some(collider) => {
                                commands.entity(child).insert(collider);
                            }
                            None => error!(
                                "Unable to generate collider from mesh {:?} with name {}",
                                mesh, name
                            ),
                        }
                    }
                }
            });

            commands.entity(entity).remove::<AsyncSceneCollider>();
        }
    }
}

/// Iterates over all descendants of the `entity` and applies `f`.
#[cfg(feature = "dim3")]
fn traverse_descendants(entity: Entity, children: &Query<&Children>, f: &mut impl FnMut(Entity)) {
    if let Ok(entity_children) = children.get(entity) {
        for child in entity_children.iter().copied() {
            f(child);
            traverse_descendants(child, children, f);
        }
    }
}

/// System responsible for creating new Rapier colliders from the related `bevy_rapier` components.
pub fn init_colliders(
    mut commands: Commands,
    config: Res<RapierConfiguration>,
    mut context: ResMut<RapierContext>,
    colliders: Query<ColliderComponents, Without<RapierColliderHandle>>,
    mut rigid_body_mprops: Query<&mut MassProperties>,
    parent_query: Query<(&Parent, Option<&Transform>)>,
) {
    let context = &mut *context;
    let scale = context.physics_scale;

    for (
        entity,
        shape,
        transform,
        sensor,
        mprops,
        active_events,
        active_hooks,
        active_collision_types,
        friction,
        restitution,
        collision_groups,
        solver_groups,
    ) in colliders.iter()
    {
        let mut scaled_shape = shape.clone();
        scaled_shape.set_scale(shape.scale / scale, config.scaled_shape_subdivision);
        let mut builder = ColliderBuilder::new(scaled_shape.raw.clone());

        if let Some(sensor) = sensor {
            builder = builder.sensor(sensor.0);
        }

        if let Some(mprops) = mprops {
            builder = match mprops {
                ColliderMassProperties::Density(density) => builder.density(*density),
                ColliderMassProperties::MassProperties(mprops) => {
                    builder.mass_properties(mprops.into_rapier(scale))
                }
            };
        }

        if let Some(active_events) = active_events {
            builder = builder.active_events((*active_events).into());
        }

        if let Some(active_hooks) = active_hooks {
            builder = builder.active_hooks((*active_hooks).into());
        }

        if let Some(active_collision_types) = active_collision_types {
            builder = builder.active_collision_types((*active_collision_types).into());
        }

        if let Some(friction) = friction {
            builder = builder
                .friction(friction.coefficient)
                .friction_combine_rule(friction.combine_rule.into());
        }

        if let Some(restitution) = restitution {
            builder = builder
                .restitution(restitution.coefficient)
                .restitution_combine_rule(restitution.combine_rule.into());
        }

        if let Some(collision_groups) = collision_groups {
            builder = builder.collision_groups((*collision_groups).into());
        }

        if let Some(solver_groups) = solver_groups {
            builder = builder.solver_groups((*solver_groups).into());
        }

        let mut body_entity = entity;
        let mut body_handle = context.entity2body.get(&body_entity).copied();
        let mut child_transform = Transform::default();
        while body_handle.is_none() {
            if let Ok((parent_entity, transform)) = parent_query.get(body_entity) {
                if let Some(transform) = transform {
                    child_transform = *transform * child_transform;
                }
                body_entity = parent_entity.0;
            } else {
                break;
            }

            body_handle = context.entity2body.get(&body_entity).copied();
        }

        builder = builder.position(utils::transform_to_iso(&child_transform.into(), scale));

        builder = builder.user_data(entity.to_bits() as u128);

        let handle = if let Some(body_handle) = body_handle {
            let handle =
                context
                    .colliders
                    .insert_with_parent(builder, body_handle, &mut context.bodies);
            if let Ok(mut mprops) = rigid_body_mprops.get_mut(body_entity) {
                // Inserting the collider changed the rigid-body’s mass properties.
                // Read them back from the engine.
                if let Some(parent_body) = context.bodies.get(body_handle) {
                    *mprops = MassProperties::from_rapier(*parent_body.mass_properties(), scale);
                }
            }
            handle
        } else {
            context.colliders.insert(builder)
        };

        commands.entity(entity).insert(RapierColliderHandle(handle));
        context.entity2collider.insert(entity, handle);
    }
}

/// System responsible for creating new Rapier rigid-bodies from the related `bevy_rapier` components.
pub fn init_rigid_bodies(
    mut commands: Commands,
    mut context: ResMut<RapierContext>,
    rigid_bodies: Query<RigidBodyComponents, Without<RapierRigidBodyHandle>>,
) {
    let scale = context.physics_scale;

    for (
        entity,
        rb,
        transform,
        vel,
        additional_mass_props,
        _mass_props,
        locked_axes,
        force,
        impulse,
        gravity_scale,
        ccd,
        dominance,
        sleep,
        damping,
    ) in rigid_bodies.iter()
    {
        let mut builder = RigidBodyBuilder::new((*rb).into());
        if let Some(transform) = transform {
            builder = builder.position(utils::transform_to_iso(transform, scale));
        }

        #[allow(clippy::useless_conversion)] // Need to convert if dim3 enabled
        if let Some(vel) = vel {
            builder = builder
                .linvel((vel.linvel / scale).into())
                .angvel(vel.angvel.into());
        }

        if let Some(locked_axes) = locked_axes {
            builder = builder.locked_axes((*locked_axes).into())
        }

        if let Some(gravity_scale) = gravity_scale {
            builder = builder.gravity_scale(gravity_scale.0);
        }

        if let Some(ccd) = ccd {
            builder = builder.ccd_enabled(ccd.enabled)
        }

        if let Some(dominance) = dominance {
            builder = builder.dominance_group(dominance.groups)
        }

        if let Some(sleep) = sleep {
            builder = builder.sleeping(sleep.sleeping);
        }

        if let Some(damping) = damping {
            builder = builder
                .linear_damping(damping.linear_damping)
                .angular_damping(damping.angular_damping);
        }

        if let Some(mprops) = additional_mass_props {
            builder = builder.additional_mass_properties(mprops.0.into_rapier(scale));
        }

        builder = builder.user_data(entity.to_bits() as u128);

        let mut rb = builder.build();

        #[allow(clippy::useless_conversion)] // Need to convert if dim3 enabled
        if let Some(force) = force {
            rb.add_force((force.force / scale).into(), false);
            rb.add_torque(force.torque.into(), false);
        }

        // TODO: will this apply the impulse twice (once here, and once in apply_rigid_body_user_changes)
        //       in some scenarios?
        #[allow(clippy::useless_conversion)] // Need to convert if dim3 enabled
        if let Some(impulse) = impulse {
            rb.apply_impulse((impulse.impulse / scale).into(), false);
            rb.apply_torque_impulse(impulse.torque_impulse.into(), false);
        }

        if let Some(sleep) = sleep {
            let activation = rb.activation_mut();
            activation.linear_threshold = sleep.linear_threshold;
            activation.angular_threshold = sleep.angular_threshold;
        }

        let handle = context.bodies.insert(builder);
        commands
            .entity(entity)
            .insert(RapierRigidBodyHandle(handle));
        context.entity2body.insert(entity, handle);

        if let Some(transform) = transform {
            context.last_body_transform_set.insert(handle, *transform);
        }
    }
}

/// System responsible for creating new Rapier joints from the related `bevy_rapier` components.
pub fn init_joints(
    mut commands: Commands,
    mut context: ResMut<RapierContext>,
    impulse_joints: Query<(Entity, &ImpulseJoint), Without<RapierImpulseJointHandle>>,
    multibody_joints: Query<(Entity, &MultibodyJoint), Without<RapierMultibodyJointHandle>>,
    parent_query: Query<&Parent>,
) {
    let context = &mut *context;
    let scale = context.physics_scale;

    for (entity, joint) in impulse_joints.iter() {
        let mut target = None;
        let mut body_entity = entity;
        while target.is_none() {
            target = context.entity2body.get(&body_entity).copied();
            if let Ok(parent_entity) = parent_query.get(body_entity) {
                body_entity = parent_entity.0;
            } else {
                break;
            }
        }

        if let (Some(target), Some(source)) = (target, context.entity2body.get(&joint.parent)) {
            let handle =
                context
                    .impulse_joints
                    .insert(*source, target, joint.data.into_rapier(scale), true);
            commands
                .entity(entity)
                .insert(RapierImpulseJointHandle(handle));
            context.entity2impulse_joint.insert(entity, handle);
        }
    }

    for (entity, joint) in multibody_joints.iter() {
        let target = context.entity2body.get(&entity);

        if let (Some(target), Some(source)) = (target, context.entity2body.get(&joint.parent)) {
            if let Some(handle) = context.multibody_joints.insert(
                *source,
                *target,
                joint.data.into_rapier(scale),
                true,
            ) {
                commands
                    .entity(entity)
                    .insert(RapierMultibodyJointHandle(handle));
                context.entity2multibody_joint.insert(entity, handle);
            } else {
                error!("Failed to create multibody joint: loop detected.")
            }
        }
    }
}

/// System responsible for removing from Rapier the rigid-bodies/colliders/joints which had
/// their related `bevy_rapier` components removed by the user (through component removal or
/// despawn).
pub fn sync_removals(
    mut commands: Commands,
    mut context: ResMut<RapierContext>,
    removed_bodies: RemovedComponents<RapierRigidBodyHandle>,
    removed_colliders: RemovedComponents<RapierColliderHandle>,
    removed_impulse_joints: RemovedComponents<RapierImpulseJointHandle>,
    removed_multibody_joints: RemovedComponents<RapierMultibodyJointHandle>,
    orphan_bodies: Query<Entity, (With<RapierRigidBodyHandle>, Without<RigidBody>)>,
    orphan_colliders: Query<Entity, (With<RapierColliderHandle>, Without<Collider>)>,
    orphan_impulse_joints: Query<Entity, (With<RapierImpulseJointHandle>, Without<ImpulseJoint>)>,
    orphan_multibody_joints: Query<
        Entity,
        (With<RapierMultibodyJointHandle>, Without<MultibodyJoint>),
    >,
) {
    /*
     * Rigid-bodies removal detection.
     */
    let context = &mut *context;
    for entity in removed_bodies.iter() {
        if let Some(handle) = context.entity2body.remove(&entity) {
            let _ = context.last_body_transform_set.remove(&handle);
            context.bodies.remove(
                handle,
                &mut context.islands,
                &mut context.colliders,
                &mut context.impulse_joints,
                &mut context.multibody_joints,
                false,
            );
        }
    }

    let context = &mut *context;
    for entity in orphan_bodies.iter() {
        if let Some(handle) = context.entity2body.remove(&entity) {
            let _ = context.last_body_transform_set.remove(&handle);
            context.bodies.remove(
                handle,
                &mut context.islands,
                &mut context.colliders,
                &mut context.impulse_joints,
                &mut context.multibody_joints,
                false,
            );
        }
        commands.entity(entity).remove::<RapierRigidBodyHandle>();
    }

    /*
     * Collider removal detection.
     */
    for entity in removed_colliders.iter() {
        if let Some(handle) = context.entity2collider.remove(&entity) {
            context
                .colliders
                .remove(handle, &mut context.islands, &mut context.bodies, true);
            context.deleted_colliders.insert(handle, entity);
        }
    }

    for entity in orphan_colliders.iter() {
        if let Some(handle) = context.entity2collider.remove(&entity) {
            context
                .colliders
                .remove(handle, &mut context.islands, &mut context.bodies, true);
            context.deleted_colliders.insert(handle, entity);
        }
        commands.entity(entity).remove::<RapierColliderHandle>();
    }

    /*
     * Impulse joint removal detection.
     */
    for entity in removed_impulse_joints.iter() {
        if let Some(handle) = context.entity2impulse_joint.remove(&entity) {
            context.impulse_joints.remove(handle, true);
        }
    }

    for entity in orphan_impulse_joints.iter() {
        if let Some(handle) = context.entity2impulse_joint.remove(&entity) {
            context.impulse_joints.remove(handle, true);
        }
        commands.entity(entity).remove::<RapierImpulseJointHandle>();
    }

    /*
     * Multibody joint removal detection.
     */
    for entity in removed_multibody_joints.iter() {
        if let Some(handle) = context.entity2multibody_joint.remove(&entity) {
            context.multibody_joints.remove(handle, true);
        }
    }

    for entity in orphan_multibody_joints.iter() {
        if let Some(handle) = context.entity2multibody_joint.remove(&entity) {
            context.multibody_joints.remove(handle, true);
        }
        commands
            .entity(entity)
            .remove::<RapierMultibodyJointHandle>();
    }

    // TODO: update mass props after collider removal.
    // TODO: what about removing forces?
}

/// Adds entity to [`CollidingEntities`] on starting collision and removes from it when the
/// collision ends.
pub fn update_colliding_entities(
    mut collision_events: EventReader<CollisionEvent>,
    mut colliding_entities: Query<&mut CollidingEntities>,
) {
    for event in collision_events.iter() {
        match event.to_owned() {
            CollisionEvent::Started(entity1, entity2, _) => {
                if let Ok(mut entities) = colliding_entities.get_mut(entity1) {
                    entities.0.insert(entity2);
                }
                if let Ok(mut entities) = colliding_entities.get_mut(entity2) {
                    entities.0.insert(entity1);
                }
            }
            CollisionEvent::Stopped(entity1, entity2, _) => {
                if let Ok(mut entities) = colliding_entities.get_mut(entity1) {
                    entities.0.remove(&entity2);
                }
                if let Ok(mut entities) = colliding_entities.get_mut(entity2) {
                    entities.0.remove(&entity1);
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    #[cfg(feature = "dim3")]
    use bevy::prelude::shape::{Capsule, Cube};
    use bevy::{
        asset::AssetPlugin,
        core::CorePlugin,
        ecs::event::Events,
        render::{settings::WgpuSettings, RenderPlugin},
        scene::ScenePlugin,
        window::WindowPlugin,
    };
    use std::f32::consts::PI;

    use super::*;
    use crate::plugin::{NoUserData, RapierPhysicsPlugin};
    #[cfg(feature = "dim3")]
    use crate::prelude::ComputedColliderShape;

    #[test]
    fn colliding_entities_updates() {
        let mut app = App::new();
        app.add_event::<CollisionEvent>()
            .add_system(update_colliding_entities);

        let entity1 = app.world.spawn().insert(CollidingEntities::default()).id();
        let entity2 = app.world.spawn().insert(CollidingEntities::default()).id();

        let mut collision_events = app
            .world
            .get_resource_mut::<Events<CollisionEvent>>()
            .unwrap();
        collision_events.send(CollisionEvent::Started(
            entity1,
            entity2,
            CollisionEventFlags::SENSOR,
        ));

        app.update();

        let colliding_entities1 = app
            .world
            .entity(entity1)
            .get::<CollidingEntities>()
            .unwrap();
        assert_eq!(
            colliding_entities1.len(),
            1,
            "There should be one colliding entity"
        );
        assert_eq!(
            colliding_entities1.iter().next().unwrap(),
            entity2,
            "Colliding entity should be equal to the second entity"
        );

        let colliding_entities2 = app
            .world
            .entity(entity2)
            .get::<CollidingEntities>()
            .unwrap();
        assert_eq!(
            colliding_entities2.len(),
            1,
            "There should be one colliding entity"
        );
        assert_eq!(
            colliding_entities2.iter().next().unwrap(),
            entity1,
            "Colliding entity should be equal to the first entity"
        );

        let mut collision_events = app
            .world
            .get_resource_mut::<Events<CollisionEvent>>()
            .unwrap();
        collision_events.send(CollisionEvent::Stopped(
            entity1,
            entity2,
            CollisionEventFlags::SENSOR,
        ));

        app.update();

        let colliding_entities1 = app
            .world
            .entity(entity1)
            .get::<CollidingEntities>()
            .unwrap();
        assert!(
            colliding_entities1.is_empty(),
            "Colliding entity should be removed from the CollidingEntities component when the collision ends"
        );

        let colliding_entities2 = app
            .world
            .entity(entity2)
            .get::<CollidingEntities>()
            .unwrap();
        assert!(
            colliding_entities2.is_empty(),
            "Colliding entity should be removed from the CollidingEntities component when the collision ends"
        );
    }

    #[test]
    #[cfg(feature = "dim3")]
    fn async_collider_initializes() {
        let mut app = App::new();
        app.add_plugin(HeadlessRenderPlugin)
            .add_system(init_async_colliders);

        let mut meshes = app.world.resource_mut::<Assets<Mesh>>();
        let cube = meshes.add(Cube::default().into());

        let entity = app
            .world
            .spawn()
            .insert(AsyncCollider {
                handle: cube,
                shape: ComputedColliderShape::TriMesh,
            })
            .id();

        app.update();

        let entity = app.world.entity(entity);
        assert!(
            entity.get::<Collider>().is_some(),
            "Collider component should be added"
        );
        assert!(
            entity.get::<AsyncCollider>().is_none(),
            "AsyncCollider component should be removed after Collider component creation"
        );
    }

    #[test]
    #[cfg(feature = "dim3")]
    fn async_scene_collider_initializes() {
        let mut app = App::new();
        app.add_plugin(HeadlessRenderPlugin)
            .add_system(init_async_scene_colliders);

        let mut meshes = app.world.resource_mut::<Assets<Mesh>>();
        let cube_handle = meshes.add(Cube::default().into());
        let capsule_handle = meshes.add(Capsule::default().into());
        let cube = app
            .world
            .spawn()
            .insert(Name::new("Cube"))
            .insert(cube_handle)
            .id();
        let capsule = app
            .world
            .spawn()
            .insert(Name::new("Capsule"))
            .insert(capsule_handle)
            .id();

        let mut scenes = app.world.resource_mut::<Assets<Scene>>();
        let scene = scenes.add(Scene::new(World::new()));

        let mut named_shapes = bevy::utils::HashMap::new();
        named_shapes.insert("Capsule".to_string(), None);
        let parent = app
            .world
            .spawn()
            .insert(AsyncSceneCollider {
                handle: scene,
                shape: Some(ComputedColliderShape::TriMesh),
                named_shapes,
            })
            .push_children(&[cube, capsule])
            .id();

        app.update();

        assert!(
            app.world.entity(cube).get::<Collider>().is_some(),
            "Collider component should be added for cube"
        );
        assert!(
            app.world.entity(capsule).get::<Collider>().is_none(),
            "Collider component shouldn't be added for capsule"
        );
        assert!(
            app.world.entity(parent).get::<AsyncCollider>().is_none(),
            "AsyncSceneCollider component should be removed after Collider components creation"
        );
    }

    #[test]
    fn transform_propagation() {
        let mut app = App::new();
        app.add_plugin(HeadlessRenderPlugin)
            .add_plugin(TransformPlugin)
            .add_plugin(RapierPhysicsPlugin::<NoUserData>::default());

        let zero = (Transform::default(), Transform::default());

        let different = (
            Transform {
                translation: Vec3::X * 10.0,
                rotation: Quat::from_rotation_x(PI),
                ..Default::default()
            },
            Transform {
                translation: Vec3::Y * 10.0,
                rotation: Quat::from_rotation_x(PI),
                ..Default::default()
            },
        );

        let same = (different.0, different.0);

        for (child_transform, parent_transform) in [zero, same, different] {
            let child = app
                .world
                .spawn()
                .insert_bundle(TransformBundle::from(child_transform))
                .insert(RigidBody::Fixed)
                .insert(Collider::ball(1.0))
                .id();

            app.world
                .spawn()
                .insert_bundle(TransformBundle::from(parent_transform))
                .push_children(&[child]);

            app.update();

            let child_transform = app.world.entity(child).get::<GlobalTransform>().unwrap();
            let context = app.world.resource::<RapierContext>();
            let child_handle = context.entity2body[&child];
            let child_body = context.bodies.get(child_handle).unwrap();
            let body_transform =
                utils::iso_to_transform(child_body.position(), context.physics_scale);
            assert_eq!(
                body_transform, *child_transform,
                "Collider transform should have have global rotation and translation"
            );
        }
    }

    #[test]
    fn transform_propagation2() {
        let mut app = App::new();
        app.add_plugin(HeadlessRenderPlugin)
            .add_plugin(TransformPlugin)
            .add_plugin(RapierPhysicsPlugin::<NoUserData>::default());

        let zero = (Transform::default(), Transform::default());

        let different = (
            Transform {
                translation: Vec3::X * 10.0,
                rotation: Quat::from_rotation_x(PI), // TODO: Why doesn't this work?
                ..Default::default()
            },
            Transform {
                translation: Vec3::Y * 10.0,
                rotation: Quat::from_rotation_x(PI),
                ..Default::default()
            },
        );

        let same = (different.0, different.0);

        for (child_transform, parent_transform) in [zero, same, different] {
            let child = app
                .world
                .spawn()
                .insert_bundle(TransformBundle::from(child_transform))
                .insert(Collider::ball(1.0))
                .id();

            let parent = app.world
                .spawn()
                .insert_bundle(TransformBundle::from(parent_transform))
                .insert(RigidBody::Fixed)
                .push_children(&[child])
                .id();

            app.update();

            let child_transform = app.world.entity(child).get::<GlobalTransform>().unwrap();
            let context = app.world.resource::<RapierContext>();
            let parent_handle = context.entity2body[&parent];
            let parent_body = context.bodies.get(parent_handle).unwrap();
            let child_collider_handle = parent_body.colliders()[0];
            let child_collider = context.colliders.get(child_collider_handle).unwrap();
            let body_transform =
                utils::iso_to_transform(child_collider.position(), context.physics_scale);
            assert_eq!(
                body_transform, *child_transform,
                "Collider transform should have have global rotation and translation"
            );
        }
    }

    // Allows run tests for systems containing rendering related things without GPU
    struct HeadlessRenderPlugin;

    impl Plugin for HeadlessRenderPlugin {
        fn build(&self, app: &mut App) {
            app.insert_resource(WgpuSettings {
                backends: None,
                ..WgpuSettings::default()
            })
            .add_plugin(CorePlugin::default())
            .add_plugin(WindowPlugin::default())
            .add_plugin(AssetPlugin::default())
            .add_plugin(ScenePlugin::default())
            .add_plugin(RenderPlugin::default());
        }
    }
}
