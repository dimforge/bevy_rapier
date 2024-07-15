use crate::dynamics::RapierRigidBodyHandle;
use crate::plugin::{configuration::TimestepMode, RapierConfiguration, RapierContext};
use crate::{dynamics::RigidBody, plugin::configuration::SimulationToRenderTime};
use crate::{prelude::*, utils};
use bevy::prelude::*;
use rapier::dynamics::{RigidBodyBuilder, RigidBodyHandle, RigidBodyType};
use std::collections::HashMap;

/// Components that will be updated after a physics step.
pub type RigidBodyWritebackComponents<'a> = (
    &'a RapierRigidBodyHandle,
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
    (Option<&'a Ccd>, Option<&'a SoftCcd>),
    Option<&'a Dominance>,
    Option<&'a Sleeping>,
    Option<&'a Damping>,
    Option<&'a RigidBodyDisabled>,
    Option<&'a AdditionalSolverIterations>,
);

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
    (changed_ccd, changed_soft_ccd): (
        Query<(&RapierRigidBodyHandle, &Ccd), Changed<Ccd>>,
        Query<(&RapierRigidBodyHandle, &SoftCcd), Changed<SoftCcd>>,
    ),
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

    // Deal with sleeping first, because other changes may then wake-up the
    // rigid-body again.
    for (handle, sleeping) in changed_sleeping.iter() {
        if let Some(rb) = context.bodies.get_mut(handle.0) {
            let activation = rb.activation_mut();
            activation.normalized_linear_threshold = sleeping.normalized_linear_threshold;
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
                        ));
                        context
                            .last_body_transform_set
                            .insert(handle.0, *global_transform);
                    }
                }
                _ => {
                    if transform_changed == Some(true) {
                        rb.set_position(
                            utils::transform_to_iso(&global_transform.compute_transform()),
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
            rb.set_linvel(velocity.linvel.into(), true);
            #[allow(clippy::useless_conversion)] // Need to convert if dim3 enabled
            rb.set_angvel(velocity.angvel.into(), true);
        }
    }

    for (entity, handle, mprops) in changed_additional_mass_props.iter() {
        if let Some(rb) = context.bodies.get_mut(handle.0) {
            match mprops {
                AdditionalMassProperties::MassProperties(mprops) => {
                    rb.set_additional_mass_properties(mprops.into_rapier(), true);
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
            rb.add_force(forces.force.into(), true);
            #[allow(clippy::useless_conversion)] // Need to convert if dim3 enabled
            rb.add_torque(forces.torque.into(), true);
        }
    }

    for (handle, mut impulses) in changed_impulses.iter_mut() {
        if let Some(rb) = context.bodies.get_mut(handle.0) {
            rb.apply_impulse(impulses.impulse.into(), true);
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

    for (handle, soft_ccd) in changed_soft_ccd.iter() {
        if let Some(rb) = context.bodies.get_mut(handle.0) {
            rb.set_soft_ccd_prediction(soft_ccd.prediction);
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

/// System responsible for writing the result of the last simulation step into our `bevy_rapier`
/// components and the [`GlobalTransform`] component.
pub fn writeback_rigid_bodies(
    mut context: ResMut<RapierContext>,
    config: Res<RapierConfiguration>,
    sim_to_render_time: Res<SimulationToRenderTime>,
    global_transforms: Query<&GlobalTransform>,
    mut writeback: Query<
        RigidBodyWritebackComponents,
        (With<RigidBody>, Without<RigidBodyDisabled>),
    >,
) {
    let context = &mut *context;

    if config.physics_pipeline_active {
        for (handle, parent, transform, mut interpolation, mut velocity, mut sleeping) in
            writeback.iter_mut()
        {
            let handle = handle.0;

            // TODO: do this the other way round: iterate through Rapier’s RigidBodySet on the active bodies,
            // and update the components accordingly. That way, we don’t have to iterate through the entities that weren’t changed
            // by physics (for example because they are sleeping).
            if let Some(rb) = context.bodies.get(handle) {
                let mut interpolated_pos = utils::iso_to_transform(rb.position());

                if let TimestepMode::Interpolated { dt, .. } = config.timestep_mode {
                    if let Some(interpolation) = interpolation.as_deref_mut() {
                        if interpolation.end.is_none() {
                            interpolation.end = Some(*rb.position());
                        }

                        if let Some(interpolated) =
                            interpolation.lerp_slerp((dt + sim_to_render_time.diff) / dt)
                        {
                            interpolated_pos = utils::iso_to_transform(&interpolated);
                        }
                    }
                }

                if let Some(mut transform) = transform {
                    // NOTE: Rapier's `RigidBody` doesn't know its own scale as it is encoded
                    //       directly within its collider, so we have to retrieve it from
                    //       the scale of its bevy transform.
                    interpolated_pos = interpolated_pos.with_scale(transform.scale);

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
                        linvel: (*rb.linvel()).into(),
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

/// System responsible for creating new Rapier rigid-bodies from the related `bevy_rapier` components.
pub fn init_rigid_bodies(
    mut commands: Commands,
    mut context: ResMut<RapierContext>,
    rigid_bodies: Query<RigidBodyComponents, Without<RapierRigidBodyHandle>>,
) {
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
        (ccd, soft_ccd),
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
            builder = builder.position(utils::transform_to_iso(&transform.compute_transform()));
        }

        #[allow(clippy::useless_conversion)] // Need to convert if dim3 enabled
        if let Some(vel) = vel {
            builder = builder.linvel(vel.linvel.into()).angvel(vel.angvel.into());
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

        if let Some(soft_ccd) = soft_ccd {
            builder = builder.soft_ccd_prediction(soft_ccd.prediction)
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
                    builder.additional_mass_properties(mprops.into_rapier())
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
            rb.add_force(force.force.into(), false);
            rb.add_torque(force.torque.into(), false);
        }

        // NOTE: we can’t apply impulses yet at this point because
        //       the rigid-body’s mass isn’t up-to-date yet (its
        //       attached colliders, if any, haven’t been created yet).

        if let Some(sleep) = sleep {
            let activation = rb.activation_mut();
            activation.normalized_linear_threshold = sleep.normalized_linear_threshold;
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
            rb.apply_impulse(impulse.impulse.into(), false);

            #[allow(clippy::useless_conversion)] // Need to convert if dim3 enabled
            rb.apply_torque_impulse(impulse.torque_impulse.into(), false);

            impulse.reset();
        }
    }
}
