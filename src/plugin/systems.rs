//! Systems responsible for interfacing our Bevy components with the Rapier physics engine.

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
use crate::plugin::{RapierConfiguration, RapierContext};
use crate::prelude::{
    AdditionalSolverIterations, BevyPhysicsHooks, BevyPhysicsHooksAdapter, CollidingEntities,
    KinematicCharacterController, KinematicCharacterControllerOutput, MassModifiedEvent,
    RigidBodyDisabled,
};
use crate::utils;
use bevy::ecs::system::{StaticSystemParam, SystemParamItem};
use bevy::prelude::*;
use rapier::{math::Real, prelude::*};
use std::collections::HashMap;

#[cfg(all(feature = "dim3", feature = "async-collider"))]
use {
    crate::prelude::{AsyncCollider, AsyncSceneCollider},
    bevy::scene::SceneInstance,
};

use crate::control::CharacterCollision;
#[cfg(feature = "dim2")]
use bevy::math::Vec3Swizzles;

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
    Option<&'a ReadMassProperties>,
    Option<&'a LockedAxes>,
    Option<&'a ExternalForce>,
    Option<&'a GravityScale>,
    Option<&'a Ccd>,
    Option<&'a Dominance>,
    Option<&'a Sleeping>,
    Option<&'a Damping>,
    Option<&'a RigidBodyDisabled>,
    Option<&'a AdditionalSolverIterations>,
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
    Option<&'a CollisionGroups>,
    Option<&'a SolverGroups>,
    Option<&'a ContactForceEventThreshold>,
    Option<&'a ColliderDisabled>,
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
            Some(ColliderScale::Relative(scale)) => {
                *scale * transform.compute_transform().scale.xy()
            }
            None => transform.compute_transform().scale.xy(),
        };
        #[cfg(feature = "dim3")]
        let effective_scale = match custom_scale {
            Some(ColliderScale::Absolute(scale)) => *scale,
            Some(ColliderScale::Relative(scale)) => *scale * transform.compute_transform().scale,
            None => transform.compute_transform().scale,
        };

        if shape.scale != crate::geometry::get_snapped_scale(effective_scale) {
            shape.set_scale(effective_scale, config.scaled_shape_subdivision);
        }
    }
}

/// System responsible for applying changes the user made to a collider-related component.
pub fn apply_collider_user_changes(
    mut context: ResMut<RapierContext>,
    config: Res<RapierConfiguration>,
    (changed_collider_transforms, parent_query, transform_query): (
        Query<
            (Entity, &RapierColliderHandle, &GlobalTransform),
            (Without<RapierRigidBodyHandle>, Changed<GlobalTransform>),
        >,
        Query<&Parent>,
        Query<&Transform>,
    ),

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
    changed_disabled: Query<(&RapierColliderHandle, &ColliderDisabled), Changed<ColliderDisabled>>,
    changed_contact_force_threshold: Query<
        (&RapierColliderHandle, &ContactForceEventThreshold),
        Changed<ContactForceEventThreshold>,
    >,
    changed_collider_mass_props: Query<
        (&RapierColliderHandle, &ColliderMassProperties),
        Changed<ColliderMassProperties>,
    >,

    mut mass_modified: EventWriter<MassModifiedEvent>,
) {
    let scale = context.physics_scale;

    for (entity, handle, transform) in changed_collider_transforms.iter() {
        if context.collider_parent(entity).is_some() {
            let (_, collider_position) =
                collider_offset(entity, &context, &parent_query, &transform_query);

            if let Some(co) = context.colliders.get_mut(handle.0) {
                co.set_position_wrt_parent(utils::transform_to_iso(&collider_position, scale));
            }
        } else if let Some(co) = context.colliders.get_mut(handle.0) {
            co.set_position(utils::transform_to_iso(
                &transform.compute_transform(),
                scale,
            ))
        }
    }

    for (handle, shape) in changed_shapes.iter() {
        if let Some(co) = context.colliders.get_mut(handle.0) {
            let mut scaled_shape = shape.clone();
            scaled_shape.set_scale(shape.scale / scale, config.scaled_shape_subdivision);
            co.set_shape(scaled_shape.raw.clone());

            if let Some(body) = co.parent() {
                if let Some(body_entity) = context.rigid_body_entity(body) {
                    mass_modified.send(body_entity.into());
                }
            }
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

    for (handle, _) in changed_sensors.iter() {
        if let Some(co) = context.colliders.get_mut(handle.0) {
            co.set_sensor(true);
        }
    }

    for (handle, _) in changed_disabled.iter() {
        if let Some(co) = context.colliders.get_mut(handle.0) {
            co.set_enabled(false);
        }
    }

    for (handle, threshold) in changed_contact_force_threshold.iter() {
        if let Some(co) = context.colliders.get_mut(handle.0) {
            co.set_contact_force_event_threshold(threshold.0);
        }
    }

    for (handle, mprops) in changed_collider_mass_props.iter() {
        if let Some(co) = context.colliders.get_mut(handle.0) {
            match mprops {
                ColliderMassProperties::Density(density) => co.set_density(*density),
                ColliderMassProperties::Mass(mass) => co.set_mass(*mass),
                ColliderMassProperties::MassProperties(mprops) => {
                    co.set_mass_properties(mprops.into_rapier(scale))
                }
            }

            if let Some(body) = co.parent() {
                if let Some(body_entity) = context.rigid_body_entity(body) {
                    mass_modified.send(body_entity.into());
                }
            }
        }
    }
}

/// System responsible for applying changes the user made to a rigid-body-related component.
pub fn apply_rigid_body_user_changes(
    mut context: ResMut<RapierContext>,
    config: Res<RapierConfiguration>,
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
        (Entity, &RapierRigidBodyHandle, &AdditionalMassProperties),
        Changed<AdditionalMassProperties>,
    >,
    changed_locked_axes: Query<(&RapierRigidBodyHandle, &LockedAxes), Changed<LockedAxes>>,
    changed_forces: Query<(&RapierRigidBodyHandle, &ExternalForce), Changed<ExternalForce>>,
    mut changed_impulses: Query<
        (&RapierRigidBodyHandle, &mut ExternalImpulse),
        Changed<ExternalImpulse>,
    >,
    changed_gravity_scale: Query<(&RapierRigidBodyHandle, &GravityScale), Changed<GravityScale>>,
    changed_ccd: Query<(&RapierRigidBodyHandle, &Ccd), Changed<Ccd>>,
    changed_dominance: Query<(&RapierRigidBodyHandle, &Dominance), Changed<Dominance>>,
    changed_sleeping: Query<(&RapierRigidBodyHandle, &Sleeping), Changed<Sleeping>>,
    changed_damping: Query<(&RapierRigidBodyHandle, &Damping), Changed<Damping>>,
    (changed_disabled, changed_additional_solver_iterations): (
        Query<(&RapierRigidBodyHandle, &RigidBodyDisabled), Changed<RigidBodyDisabled>>,
        Query<
            (&RapierRigidBodyHandle, &AdditionalSolverIterations),
            Changed<AdditionalSolverIterations>,
        >,
    ),
    mut mass_modified: EventWriter<MassModifiedEvent>,
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
            rb.set_body_type((*rb_type).into(), true);
        }
    }

    // Manually checks if the transform changed.
    // This is needed for detecting if the user actually changed the rigid-body
    // transform, or if it was just the change we made in our `writeback_rigid_bodies`
    // system.
    let transform_changed_fn =
        |handle: &RigidBodyHandle,
         transform: &GlobalTransform,
         last_transform_set: &HashMap<RigidBodyHandle, GlobalTransform>| {
            if config.force_update_from_transform_changes {
                true
            } else if let Some(prev) = last_transform_set.get(handle) {
                *prev != *transform
            } else {
                true
            }
        };

    for (handle, global_transform, mut interpolation) in changed_transforms.iter_mut() {
        // Use an Option<bool> to avoid running the check twice.
        let mut transform_changed = None;

        if let Some(interpolation) = interpolation.as_deref_mut() {
            transform_changed = transform_changed.or_else(|| {
                Some(transform_changed_fn(
                    &handle.0,
                    global_transform,
                    &context.last_body_transform_set,
                ))
            });

            if transform_changed == Some(true) {
                // Reset the interpolation so we don’t overwrite
                // the user’s input.
                interpolation.start = None;
                interpolation.end = None;
            }
        }

        if let Some(rb) = context.bodies.get_mut(handle.0) {
            transform_changed = transform_changed.or_else(|| {
                Some(transform_changed_fn(
                    &handle.0,
                    global_transform,
                    &context.last_body_transform_set,
                ))
            });

            match rb.body_type() {
                RigidBodyType::KinematicPositionBased => {
                    if transform_changed == Some(true) {
                        rb.set_next_kinematic_position(utils::transform_to_iso(
                            &global_transform.compute_transform(),
                            scale,
                        ));
                        context
                            .last_body_transform_set
                            .insert(handle.0, *global_transform);
                    }
                }
                _ => {
                    if transform_changed == Some(true) {
                        rb.set_position(
                            utils::transform_to_iso(&global_transform.compute_transform(), scale),
                            true,
                        );
                        context
                            .last_body_transform_set
                            .insert(handle.0, *global_transform);
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

    for (entity, handle, mprops) in changed_additional_mass_props.iter() {
        if let Some(rb) = context.bodies.get_mut(handle.0) {
            match mprops {
                AdditionalMassProperties::MassProperties(mprops) => {
                    rb.set_additional_mass_properties(mprops.into_rapier(scale), true);
                }
                AdditionalMassProperties::Mass(mass) => {
                    rb.set_additional_mass(*mass, true);
                }
            }

            mass_modified.send(entity.into());
        }
    }

    for (handle, additional_solver_iters) in changed_additional_solver_iterations.iter() {
        if let Some(rb) = context.bodies.get_mut(handle.0) {
            rb.set_additional_solver_iterations(additional_solver_iters.0);
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

    for (handle, mut impulses) in changed_impulses.iter_mut() {
        if let Some(rb) = context.bodies.get_mut(handle.0) {
            rb.apply_impulse((impulses.impulse / scale).into(), true);
            #[allow(clippy::useless_conversion)] // Need to convert if dim3 enabled
            rb.apply_torque_impulse(impulses.torque_impulse.into(), true);
            impulses.reset();
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

    for (handle, _) in changed_disabled.iter() {
        if let Some(co) = context.bodies.get_mut(handle.0) {
            co.set_enabled(false);
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
    fixed_time: Res<Time<Fixed>>,
    global_transforms: Query<&GlobalTransform>,
    mut writeback: Query<
        RigidBodyWritebackComponents,
        (With<RigidBody>, Without<RigidBodyDisabled>),
    >,
) {
    let context = &mut *context;
    let scale = context.physics_scale;
    let lerp_percentage = fixed_time.overstep_percentage();

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

                    if let Some(interpolation) = interpolation.as_deref_mut() {
                        if let Some(interpolated) = interpolation.lerp_slerp(lerp_percentage) {
                            interpolated_pos = utils::iso_to_transform(&interpolated, scale);
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
                            // We need to compute the new local transform such that:
                            // curr_parent_global_transform * new_transform = interpolated_pos
                            // new_transform = curr_parent_global_transform.inverse() * interpolated_pos
                            let (_, inverse_parent_rotation, inverse_parent_translation) =
                                parent_global_transform
                                    .affine()
                                    .inverse()
                                    .to_scale_rotation_translation();
                            let new_rotation = inverse_parent_rotation * interpolated_pos.rotation;

                            #[allow(unused_mut)] // mut is needed in 2D but not in 3D.
                            let mut new_translation = inverse_parent_rotation
                                * interpolated_pos.translation
                                + inverse_parent_translation;

                            // In 2D, preserve the transform `z` component that may have been set by the user
                            #[cfg(feature = "dim2")]
                            {
                                new_translation.z = transform.translation.z;
                            }

                            if transform.rotation != new_rotation
                                || transform.translation != new_translation
                            {
                                // NOTE: we write the new value only if there was an
                                //       actual change, in order to not trigger bevy’s
                                //       change tracking when the values didn’t change.
                                transform.rotation = new_rotation;
                                transform.translation = new_translation;
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

                            if transform.rotation != interpolated_pos.rotation
                                || transform.translation != interpolated_pos.translation
                            {
                                // NOTE: we write the new value only if there was an
                                //       actual change, in order to not trigger bevy’s
                                //       change tracking when the values didn’t change.
                                transform.rotation = interpolated_pos.rotation;
                                transform.translation = interpolated_pos.translation;
                            }

                            context
                                .last_body_transform_set
                                .insert(handle, GlobalTransform::from(interpolated_pos));
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

/// System responsible for writing updated mass properties back into the [`ReadMassProperties`] component.
pub fn writeback_mass_properties(
    mut context: ResMut<RapierContext>,
    config: Res<RapierConfiguration>,

    mut mass_props: Query<&mut ReadMassProperties>,
    mut mass_modified: EventReader<MassModifiedEvent>,
) {
    let context = &mut *context;
    let scale = context.physics_scale;

    if config.physics_pipeline_active {
        for entity in mass_modified.read() {
            if let Some(handle) = context.entity2body.get(entity).copied() {
                if let Some(rb) = context.bodies.get(handle) {
                    if let Ok(mut mass_props) = mass_props.get_mut(**entity) {
                        let new_mass_props =
                            MassProperties::from_rapier(rb.mass_properties().local_mprops, scale);

                        // NOTE: we write the new value only if there was an
                        //       actual change, in order to not trigger bevy’s
                        //       change tracking when the values didn’t change.
                        if mass_props.get() != &new_mass_props {
                            mass_props.set(new_mass_props);
                        }
                    }
                }
            }
        }
    }
}

/// System responsible for advancing the physics simulation, and updating the internal state
/// for scene queries.
pub fn step_simulation<Hooks>(
    mut context: ResMut<RapierContext>,
    config: Res<RapierConfiguration>,
    hooks: StaticSystemParam<Hooks>,
    time: Res<Time<Fixed>>,
    collision_events: EventWriter<CollisionEvent>,
    contact_force_events: EventWriter<ContactForceEvent>,
    interpolation_query: Query<(&RapierRigidBodyHandle, &mut TransformInterpolation)>,
) where
    Hooks: 'static + BevyPhysicsHooks,
    for<'w, 's> SystemParamItem<'w, 's, Hooks>: BevyPhysicsHooks,
{
    let context = &mut *context;
    let hooks_adapter = BevyPhysicsHooksAdapter::new(hooks.into_inner());

    if config.physics_pipeline_active {
        context.step_simulation(
            config.gravity,
            Some((collision_events, contact_force_events)),
            &hooks_adapter,
            time.delta_seconds(),
            config.substeps,
            interpolation_query,
        );
        context.deleted_colliders.clear();
    } else {
        context.propagate_modified_body_positions_to_colliders();
    }

    if config.query_pipeline_active {
        context.update_query_pipeline();
    }
}

/// System responsible for creating `Collider` components from `AsyncCollider` components if the
/// corresponding mesh has become available.
#[cfg(all(feature = "dim3", feature = "async-collider"))]
pub fn init_async_colliders(
    mut commands: Commands,
    meshes: Res<Assets<Mesh>>,
    async_colliders: Query<(Entity, &Handle<Mesh>, &AsyncCollider)>,
) {
    for (entity, mesh_handle, async_collider) in async_colliders.iter() {
        if let Some(mesh) = meshes.get(mesh_handle) {
            match Collider::from_bevy_mesh(mesh, &async_collider.0) {
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
#[cfg(all(feature = "dim3", feature = "async-collider"))]
pub fn init_async_scene_colliders(
    mut commands: Commands,
    meshes: Res<Assets<Mesh>>,
    scene_spawner: Res<SceneSpawner>,
    async_colliders: Query<(Entity, &SceneInstance, &AsyncSceneCollider)>,
    children: Query<&Children>,
    mesh_handles: Query<(&Name, &Handle<Mesh>)>,
) {
    for (scene_entity, scene_instance, async_collider) in async_colliders.iter() {
        if scene_spawner.instance_is_ready(**scene_instance) {
            for child_entity in children.iter_descendants(scene_entity) {
                if let Ok((name, handle)) = mesh_handles.get(child_entity) {
                    let shape = async_collider
                        .named_shapes
                        .get(name.as_str())
                        .unwrap_or(&async_collider.shape);
                    if let Some(shape) = shape {
                        let mesh = meshes.get(handle).unwrap(); // NOTE: Mesh is already loaded
                        match Collider::from_bevy_mesh(mesh, shape) {
                            Some(collider) => {
                                commands.entity(child_entity).insert(collider);
                            }
                            None => error!(
                                "Unable to generate collider from mesh {:?} with name {}",
                                mesh, name
                            ),
                        }
                    }
                }
            }

            commands.entity(scene_entity).remove::<AsyncSceneCollider>();
        }
    }
}

fn collider_offset(
    entity: Entity,
    context: &RapierContext,
    parent_query: &Query<&Parent>,
    transform_query: &Query<&Transform>,
) -> (Option<RigidBodyHandle>, Transform) {
    let mut body_entity = entity;
    let mut body_handle = context.entity2body.get(&body_entity).copied();
    let mut child_transform = Transform::default();
    while body_handle.is_none() {
        if let Ok(parent_entity) = parent_query.get(body_entity) {
            if let Ok(transform) = transform_query.get(body_entity) {
                child_transform = *transform * child_transform;
            }
            body_entity = parent_entity.get();
        } else {
            break;
        }

        body_handle = context.entity2body.get(&body_entity).copied();
    }

    if body_handle.is_some() {
        if let Ok(transform) = transform_query.get(body_entity) {
            let scale_transform = Transform {
                scale: transform.scale,
                ..default()
            };

            child_transform = scale_transform * child_transform;
        }
    }

    (body_handle, child_transform)
}

/// System responsible for creating new Rapier colliders from the related `bevy_rapier` components.
pub fn init_colliders(
    mut commands: Commands,
    config: Res<RapierConfiguration>,
    mut context: ResMut<RapierContext>,
    colliders: Query<(ColliderComponents, Option<&GlobalTransform>), Without<RapierColliderHandle>>,
    mut rigid_body_mprops: Query<&mut ReadMassProperties>,
    parent_query: Query<&Parent>,
    transform_query: Query<&Transform>,
) {
    let context = &mut *context;
    let physics_scale = context.physics_scale;

    for (
        (
            entity,
            shape,
            sensor,
            mprops,
            active_events,
            active_hooks,
            active_collision_types,
            friction,
            restitution,
            collision_groups,
            solver_groups,
            contact_force_event_threshold,
            disabled,
        ),
        global_transform,
    ) in colliders.iter()
    {
        let mut scaled_shape = shape.clone();
        scaled_shape.set_scale(shape.scale / physics_scale, config.scaled_shape_subdivision);
        let mut builder = ColliderBuilder::new(scaled_shape.raw.clone());

        builder = builder.sensor(sensor.is_some());
        builder = builder.enabled(disabled.is_none());

        if let Some(mprops) = mprops {
            builder = match mprops {
                ColliderMassProperties::Density(density) => builder.density(*density),
                ColliderMassProperties::Mass(mass) => builder.mass(*mass),
                ColliderMassProperties::MassProperties(mprops) => {
                    builder.mass_properties(mprops.into_rapier(physics_scale))
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

        if let Some(threshold) = contact_force_event_threshold {
            builder = builder.contact_force_event_threshold(threshold.0);
        }

        let body_entity = entity;
        let (body_handle, child_transform) =
            collider_offset(entity, context, &parent_query, &transform_query);

        builder = builder.user_data(entity.to_bits() as u128);

        let handle = if let Some(body_handle) = body_handle {
            builder = builder.position(utils::transform_to_iso(&child_transform, physics_scale));
            let handle =
                context
                    .colliders
                    .insert_with_parent(builder, body_handle, &mut context.bodies);
            if let Ok(mut mprops) = rigid_body_mprops.get_mut(body_entity) {
                // Inserting the collider changed the rigid-body’s mass properties.
                // Read them back from the engine.
                if let Some(parent_body) = context.bodies.get(body_handle) {
                    mprops.set(MassProperties::from_rapier(
                        parent_body.mass_properties().local_mprops,
                        physics_scale,
                    ));
                }
            }
            handle
        } else {
            let global_transform = global_transform.cloned().unwrap_or_default();
            builder = builder.position(utils::transform_to_iso(
                &global_transform.compute_transform(),
                physics_scale,
            ));
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
    let physics_scale = context.physics_scale;

    for (
        entity,
        rb,
        transform,
        vel,
        additional_mass_props,
        _mass_props,
        locked_axes,
        force,
        gravity_scale,
        ccd,
        dominance,
        sleep,
        damping,
        disabled,
        additional_solver_iters,
    ) in rigid_bodies.iter()
    {
        let mut builder = RigidBodyBuilder::new((*rb).into());
        builder = builder.enabled(disabled.is_none());

        if let Some(transform) = transform {
            builder = builder.position(utils::transform_to_iso(
                &transform.compute_transform(),
                physics_scale,
            ));
        }

        #[allow(clippy::useless_conversion)] // Need to convert if dim3 enabled
        if let Some(vel) = vel {
            builder = builder
                .linvel((vel.linvel / physics_scale).into())
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
            builder = match mprops {
                AdditionalMassProperties::MassProperties(mprops) => {
                    builder.additional_mass_properties(mprops.into_rapier(physics_scale))
                }
                AdditionalMassProperties::Mass(mass) => builder.additional_mass(*mass),
            };
        }

        if let Some(added_iters) = additional_solver_iters {
            builder = builder.additional_solver_iterations(added_iters.0);
        }

        builder = builder.user_data(entity.to_bits() as u128);

        let mut rb = builder.build();

        #[allow(clippy::useless_conversion)] // Need to convert if dim3 enabled
        if let Some(force) = force {
            rb.add_force((force.force / physics_scale).into(), false);
            rb.add_torque(force.torque.into(), false);
        }

        // NOTE: we can’t apply impulses yet at this point because
        //       the rigid-body’s mass isn’t up-to-date yet (its
        //       attached colliders, if any, haven’t been created yet).

        if let Some(sleep) = sleep {
            let activation = rb.activation_mut();
            activation.linear_threshold = sleep.linear_threshold;
            activation.angular_threshold = sleep.angular_threshold;
        }

        let handle = context.bodies.insert(rb);
        commands
            .entity(entity)
            .insert(RapierRigidBodyHandle(handle));
        context.entity2body.insert(entity, handle);

        if let Some(transform) = transform {
            context.last_body_transform_set.insert(handle, *transform);
        }
    }
}

/// This applies the initial impulse given to a rigid-body when it is created.
///
/// This cannot be done inside `init_rigid_bodies` because impulses require the rigid-body
/// mass to be available, which it was not because colliders were not created yet. As a
/// result, we run this system after the collider creation.
pub fn apply_initial_rigid_body_impulses(
    mut context: ResMut<RapierContext>,
    // We can’t use RapierRigidBodyHandle yet because its creation command hasn’t been
    // executed yet.
    mut init_impulses: Query<(Entity, &mut ExternalImpulse), Without<RapierRigidBodyHandle>>,
) {
    let context = &mut *context;
    let scale = context.physics_scale;

    for (entity, mut impulse) in init_impulses.iter_mut() {
        let bodies = &mut context.bodies;
        if let Some(rb) = context
            .entity2body
            .get(&entity)
            .and_then(|h| bodies.get_mut(*h))
        {
            // Make sure the mass-properties are computed.
            rb.recompute_mass_properties_from_colliders(&context.colliders);
            // Apply the impulse.
            rb.apply_impulse((impulse.impulse / scale).into(), false);

            #[allow(clippy::useless_conversion)] // Need to convert if dim3 enabled
            rb.apply_torque_impulse(impulse.torque_impulse.into(), false);

            impulse.reset();
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
                body_entity = parent_entity.get();
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
    mut removed_bodies: RemovedComponents<RapierRigidBodyHandle>,
    mut removed_colliders: RemovedComponents<RapierColliderHandle>,
    mut removed_impulse_joints: RemovedComponents<RapierImpulseJointHandle>,
    mut removed_multibody_joints: RemovedComponents<RapierMultibodyJointHandle>,
    orphan_bodies: Query<Entity, (With<RapierRigidBodyHandle>, Without<RigidBody>)>,
    orphan_colliders: Query<Entity, (With<RapierColliderHandle>, Without<Collider>)>,
    orphan_impulse_joints: Query<Entity, (With<RapierImpulseJointHandle>, Without<ImpulseJoint>)>,
    orphan_multibody_joints: Query<
        Entity,
        (With<RapierMultibodyJointHandle>, Without<MultibodyJoint>),
    >,

    mut removed_sensors: RemovedComponents<Sensor>,
    mut removed_rigid_body_disabled: RemovedComponents<RigidBodyDisabled>,
    mut removed_colliders_disabled: RemovedComponents<ColliderDisabled>,

    mut mass_modified: EventWriter<MassModifiedEvent>,
) {
    /*
     * Rigid-bodies removal detection.
     */
    let context = &mut *context;
    for entity in removed_bodies.read() {
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
    for entity in removed_colliders.read() {
        if let Some(parent) = context.collider_parent(entity) {
            mass_modified.send(parent.into());
        }

        if let Some(handle) = context.entity2collider.remove(&entity) {
            context
                .colliders
                .remove(handle, &mut context.islands, &mut context.bodies, true);
            context.deleted_colliders.insert(handle, entity);
        }
    }

    for entity in orphan_colliders.iter() {
        if let Some(parent) = context.collider_parent(entity) {
            mass_modified.send(parent.into());
        }

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
    for entity in removed_impulse_joints.read() {
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
    for entity in removed_multibody_joints.read() {
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

    /*
     * Marker components removal detection.
     */
    for entity in removed_sensors.read() {
        if let Some(handle) = context.entity2collider.get(&entity) {
            if let Some(co) = context.colliders.get_mut(*handle) {
                co.set_sensor(false);
            }
        }
    }

    for entity in removed_colliders_disabled.read() {
        if let Some(handle) = context.entity2collider.get(&entity) {
            if let Some(co) = context.colliders.get_mut(*handle) {
                co.set_enabled(true);
            }
        }
    }

    for entity in removed_rigid_body_disabled.read() {
        if let Some(handle) = context.entity2body.get(&entity) {
            if let Some(rb) = context.bodies.get_mut(*handle) {
                rb.set_enabled(true);
            }
        }
    }

    // TODO: what about removing forces?
}

/// Adds entity to [`CollidingEntities`] on starting collision and removes from it when the
/// collision ends.
pub fn update_colliding_entities(
    mut collision_events: EventReader<CollisionEvent>,
    mut colliding_entities: Query<&mut CollidingEntities>,
) {
    for event in collision_events.read() {
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

/// System responsible for applying the character controller translation to the underlying
/// collider.
pub fn update_character_controls(
    mut commands: Commands,
    config: Res<RapierConfiguration>,
    mut context: ResMut<RapierContext>,
    mut character_controllers: Query<(
        Entity,
        &mut KinematicCharacterController,
        Option<&mut KinematicCharacterControllerOutput>,
        Option<&RapierColliderHandle>,
        Option<&RapierRigidBodyHandle>,
        Option<&GlobalTransform>,
    )>,
    mut transforms: Query<&mut Transform>,
) {
    let physics_scale = context.physics_scale;
    let context = &mut *context;
    for (entity, mut controller, output, collider_handle, body_handle, glob_transform) in
        character_controllers.iter_mut()
    {
        if let (Some(raw_controller), Some(translation)) =
            (controller.to_raw(physics_scale), controller.translation)
        {
            let scaled_custom_shape =
                controller
                    .custom_shape
                    .as_ref()
                    .map(|(custom_shape, tra, rot)| {
                        // TODO: avoid the systematic scale somehow?
                        let mut scaled_shape = custom_shape.clone();
                        scaled_shape.set_scale(
                            custom_shape.scale / physics_scale,
                            config.scaled_shape_subdivision,
                        );

                        (scaled_shape, *tra / physics_scale, *rot)
                    });

            let parent_rigid_body = body_handle.map(|h| h.0).or_else(|| {
                collider_handle
                    .and_then(|h| context.colliders.get(h.0))
                    .and_then(|c| c.parent())
            });
            let entity_to_move = parent_rigid_body
                .and_then(|rb| context.rigid_body_entity(rb))
                .unwrap_or(entity);

            let (character_shape, character_pos) = if let Some((scaled_shape, tra, rot)) =
                &scaled_custom_shape
            {
                let mut shape_pos: Isometry<Real> = (*tra, *rot).into();

                if let Some(body) = body_handle.and_then(|h| context.bodies.get(h.0)) {
                    shape_pos = body.position() * shape_pos
                } else if let Some(gtransform) = glob_transform {
                    shape_pos =
                        utils::transform_to_iso(&gtransform.compute_transform(), physics_scale)
                            * shape_pos
                }

                (&*scaled_shape.raw, shape_pos)
            } else if let Some(collider) = collider_handle.and_then(|h| context.colliders.get(h.0))
            {
                (collider.shape(), *collider.position())
            } else {
                continue;
            };

            let exclude_collider = collider_handle.map(|h| h.0);

            let character_mass = controller
                .custom_mass
                .or_else(|| {
                    parent_rigid_body
                        .and_then(|h| context.bodies.get(h))
                        .map(|rb| rb.mass())
                })
                .unwrap_or(0.0);

            let mut filter = QueryFilter {
                flags: controller.filter_flags,
                groups: controller.filter_groups.map(|g| g.into()),
                exclude_collider: None,
                exclude_rigid_body: None,
                predicate: None,
            };

            if let Some(parent) = parent_rigid_body {
                filter = filter.exclude_rigid_body(parent);
            } else if let Some(excl_co) = exclude_collider {
                filter = filter.exclude_collider(excl_co)
            };

            let collisions = &mut context.character_collisions_collector;
            collisions.clear();

            let movement = raw_controller.move_shape(
                context.integration_parameters.dt,
                &context.bodies,
                &context.colliders,
                &context.query_pipeline,
                character_shape,
                &character_pos,
                (translation / physics_scale).into(),
                filter,
                |c| collisions.push(c),
            );

            if controller.apply_impulse_to_dynamic_bodies {
                for collision in &*collisions {
                    raw_controller.solve_character_collision_impulses(
                        context.integration_parameters.dt,
                        &mut context.bodies,
                        &context.colliders,
                        &context.query_pipeline,
                        character_shape,
                        character_mass,
                        collision,
                        filter,
                    )
                }
            }

            // NOTE: Update the translation of the Rapier's rigid body without editing the Bevy's transform for
            //       preventing noticing change in the [`systems::apply_rigid_body_user_changes`] for kinematic bodies.
            if let Some(body) = body_handle.and_then(|h| context.bodies.get_mut(h.0)) {
                // TODO: Use `set_next_kinematic_translation` for position based kinematic bodies? - Vixenka 29.01.2024
                body.set_translation(
                    body.translation() + movement.translation * physics_scale,
                    true,
                );
            } else if let Ok(mut transform) = transforms.get_mut(entity_to_move) {
                // I do not know if this else case is still needed, but it was existing in the original code. - Vixenka 29.01.2024
                // TODO: take the parent’s GlobalTransform rotation into account?
                transform.translation.x += movement.translation.x * physics_scale;
                transform.translation.y += movement.translation.y * physics_scale;
                #[cfg(feature = "dim3")]
                {
                    transform.translation.z += movement.translation.z * physics_scale;
                }
            }

            let converted_collisions = context
                .character_collisions_collector
                .iter()
                .filter_map(|c| CharacterCollision::from_raw(context, c));

            if let Some(mut output) = output {
                output.desired_translation = controller.translation.unwrap(); // Already takes the physics_scale into account.
                output.effective_translation = (movement.translation * physics_scale).into();
                output.grounded = movement.grounded;
                output.collisions.clear();
                output.collisions.extend(converted_collisions);
            } else {
                commands
                    .entity(entity)
                    .insert(KinematicCharacterControllerOutput {
                        desired_translation: controller.translation.unwrap(), // Already takes the physics_scale into account.
                        effective_translation: (movement.translation * physics_scale).into(),
                        grounded: movement.grounded,
                        collisions: converted_collisions.collect(),
                    });
            }

            controller.translation = None;
        }
    }
}

#[cfg(test)]
mod tests {
    #[cfg(all(feature = "dim3", feature = "async-collider"))]
    use bevy::prelude::{Capsule3d, Cuboid};
    use bevy::{
        asset::AssetPlugin,
        ecs::event::Events,
        render::{
            settings::{RenderCreation, WgpuSettings},
            RenderPlugin,
        },
        scene::ScenePlugin,
        time::TimePlugin,
        window::WindowPlugin,
    };
    use std::f32::consts::PI;

    use super::*;
    use crate::{
        plugin::{NoUserData, RapierPhysicsPlugin},
        utils::testing,
    };

    #[test]
    fn colliding_entities_updates() {
        let mut app = App::new();
        app.add_event::<CollisionEvent>()
            .add_systems(Update, update_colliding_entities);

        let entity1 = app.world.spawn(CollidingEntities::default()).id();
        let entity2 = app.world.spawn(CollidingEntities::default()).id();

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
    #[cfg(all(feature = "dim3", feature = "async-collider"))]
    fn async_collider_initializes() {
        let mut app = App::new();
        app.add_plugins(HeadlessRenderPlugin)
            .add_systems(Update, init_async_colliders);

        let mut meshes = app.world.resource_mut::<Assets<Mesh>>();
        let cube = meshes.add(Cuboid::default());

        let entity = app.world.spawn((cube, AsyncCollider::default())).id();

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
    #[cfg(all(feature = "dim3", feature = "async-collider"))]
    fn async_scene_collider_initializes() {
        let mut app = App::new();
        app.add_plugins(HeadlessRenderPlugin)
            .add_systems(PostUpdate, init_async_scene_colliders);

        let mut meshes = app.world.resource_mut::<Assets<Mesh>>();
        let cube_handle = meshes.add(Cuboid::default());
        let capsule_handle = meshes.add(Capsule3d::default());
        let cube = app.world.spawn((Name::new("Cube"), cube_handle)).id();
        let capsule = app.world.spawn((Name::new("Capsule"), capsule_handle)).id();

        let mut scenes = app.world.resource_mut::<Assets<Scene>>();
        let scene = scenes.add(Scene::new(World::new()));

        let mut named_shapes = bevy::utils::HashMap::new();
        named_shapes.insert("Capsule".to_string(), None);
        let parent = app
            .world
            .spawn((
                scene,
                AsyncSceneCollider {
                    named_shapes,
                    ..Default::default()
                },
            ))
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
        app.add_plugins((
            HeadlessRenderPlugin,
            TransformPlugin,
            TimePlugin,
            RapierPhysicsPlugin::<NoUserData>::default(),
        ));

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
                .spawn((
                    TransformBundle::from(child_transform),
                    RigidBody::Fixed,
                    Collider::ball(1.0),
                ))
                .id();

            app.world
                .spawn(TransformBundle::from(parent_transform))
                .push_children(&[child]);

            testing::step_fixed_update(&mut app);

            let child_transform = app.world.entity(child).get::<GlobalTransform>().unwrap();
            let context = app.world.resource::<RapierContext>();
            let child_handle = context.entity2body[&child];
            let child_body = context.bodies.get(child_handle).unwrap();
            let body_transform =
                utils::iso_to_transform(child_body.position(), context.physics_scale);
            assert_eq!(
                GlobalTransform::from(body_transform),
                *child_transform,
                "Collider transform should have have global rotation and translation"
            );
        }
    }

    #[test]
    fn transform_propagation2() {
        let mut app = App::new();
        app.add_plugins((
            HeadlessRenderPlugin,
            TransformPlugin,
            TimePlugin,
            RapierPhysicsPlugin::<NoUserData>::default(),
        ));

        let zero = (Transform::default(), Transform::default());

        let different = (
            Transform {
                translation: Vec3::X * 10.0,
                // NOTE: in 2D the test will fail if the rotation is wrt. an axis
                //       other than Z because 2D physics objects can’t rotate wrt.
                //       other axes.
                rotation: Quat::from_rotation_z(PI),
                ..Default::default()
            },
            Transform {
                translation: Vec3::Y * 10.0,
                rotation: Quat::from_rotation_z(PI),
                ..Default::default()
            },
        );

        let same = (different.0, different.0);

        for (child_transform, parent_transform) in [zero, same, different] {
            let child = app
                .world
                .spawn((TransformBundle::from(child_transform), Collider::ball(1.0)))
                .id();

            let parent = app
                .world
                .spawn((TransformBundle::from(parent_transform), RigidBody::Fixed))
                .push_children(&[child])
                .id();

            testing::step_fixed_update(&mut app);

            let child_transform = app
                .world
                .entity(child)
                .get::<GlobalTransform>()
                .unwrap()
                .compute_transform();
            let context = app.world.resource::<RapierContext>();
            let parent_handle = context.entity2body[&parent];
            let parent_body = context.bodies.get(parent_handle).unwrap();
            let child_collider_handle = parent_body.colliders()[0];
            let child_collider = context.colliders.get(child_collider_handle).unwrap();
            let body_transform =
                utils::iso_to_transform(child_collider.position(), context.physics_scale);
            approx::assert_relative_eq!(
                body_transform.translation,
                child_transform.translation,
                epsilon = 1.0e-5
            );

            // Adjust signs to account for the quaternion’s double covering.
            let comparison_child_rotation =
                if body_transform.rotation.w * child_transform.rotation.w < 0.0 {
                    -child_transform.rotation
                } else {
                    child_transform.rotation
                };

            approx::assert_relative_eq!(
                body_transform.rotation,
                comparison_child_rotation,
                epsilon = 1.0e-5
            );
            approx::assert_relative_eq!(body_transform.scale, child_transform.scale,);
        }
    }

    // Allows run tests for systems containing rendering related things without GPU
    struct HeadlessRenderPlugin;

    impl Plugin for HeadlessRenderPlugin {
        fn build(&self, app: &mut App) {
            app.add_plugins((
                WindowPlugin::default(),
                AssetPlugin::default(),
                ScenePlugin,
                RenderPlugin {
                    render_creation: RenderCreation::Automatic(WgpuSettings {
                        backends: None,
                        ..Default::default()
                    }),
                    ..Default::default()
                },
                ImagePlugin::default(),
            ));
        }
    }
}
