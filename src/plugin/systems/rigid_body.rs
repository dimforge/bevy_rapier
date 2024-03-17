use crate::dynamics::{
    AdditionalMassProperties, Ccd, Damping, Dominance, ExternalForce, ExternalImpulse,
    GravityScale, LockedAxes, ReadMassProperties, RigidBody, RigidBodyHandle, SleepState,
    TransformInterpolation, Velocity,
};
use crate::plugin::{RapierConfiguration, RapierContext};
use crate::prelude::{
    AdditionalSolverIterations, MassModifiedEvent, RigidBodyCreated, RigidBodyDisabled,
};
use crate::utils;
use bevy::prelude::*;
use rapier::prelude::*;
use std::collections::HashMap;

#[cfg(feature = "dim2")]
use bevy::math::Vec3Swizzles;

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
    Option<&'a SleepState>,
    Option<&'a Damping>,
    Option<&'a RigidBodyDisabled>,
    Option<&'a AdditionalSolverIterations>,
);

/// System responsible for creating new Rapier rigid-bodies from the related `bevy_rapier` components.
pub fn init_rigid_bodies(
    mut commands: Commands,
    mut context: ResMut<RapierContext>,
    rigid_bodies: Query<RigidBodyComponents, Without<RigidBodyCreated>>,
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
            builder = builder.position(utils::transform_to_iso(&transform.compute_transform()));
        }

        #[allow(clippy::useless_conversion)] // Need to convert if dim3 enabled
        if let Some(vel) = vel {
            builder = builder.linvel(vel.linvel).angvel(vel.angvel);
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
                    builder.additional_mass_properties(*mprops)
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
            rb.add_force(force.force, false);
            rb.add_torque(force.torque, false);
        }

        // NOTE: we can’t apply impulses yet at this point because
        //       the rigid-body’s mass isn’t up-to-date yet (its
        //       attached colliders, if any, haven’t been created yet).

        if let Some(sleep) = sleep {
            let activation = rb.activation_mut();
            activation.linear_threshold = sleep.linear_threshold;
            activation.angular_threshold = sleep.angular_threshold;
        }

        context.bodies.insert(entity, rb);
        commands.entity(entity).insert(RigidBodyCreated);

        if let Some(transform) = transform {
            context.last_body_transform_set.insert(entity, *transform);
        }
    }
}

/// System responsible for applying changes the user made to a rigid-body-related component.
pub fn apply_rigid_body_user_changes(
    mut context: ResMut<RapierContext>,
    config: Res<RapierConfiguration>,
    changed_rb_types: Query<(Entity, &RigidBody), Changed<RigidBody>>,
    mut changed_transforms: Query<
        (
            Entity,
            &GlobalTransform,
            Option<&mut TransformInterpolation>,
        ),
        Changed<GlobalTransform>,
    >,
    changed_velocities: Query<(Entity, &Velocity), Changed<Velocity>>,
    changed_additional_mass_props: Query<
        (Entity, &AdditionalMassProperties),
        Changed<AdditionalMassProperties>,
    >,
    changed_locked_axes: Query<(Entity, &LockedAxes), Changed<LockedAxes>>,
    changed_forces: Query<(Entity, &ExternalForce), Changed<ExternalForce>>,
    mut changed_impulses: Query<(Entity, &mut ExternalImpulse), Changed<ExternalImpulse>>,
    changed_gravity_scale: Query<(Entity, &GravityScale), Changed<GravityScale>>,
    changed_ccd: Query<(Entity, &Ccd), Changed<Ccd>>,
    changed_dominance: Query<(Entity, &Dominance), Changed<Dominance>>,
    changed_sleeping: Query<(Entity, &SleepState), Changed<SleepState>>,
    changed_damping: Query<(Entity, &Damping), Changed<Damping>>,
    (changed_disabled, changed_additional_solver_iterations): (
        Query<(Entity, &RigidBodyDisabled), Changed<RigidBodyDisabled>>,
        Query<(Entity, &AdditionalSolverIterations), Changed<AdditionalSolverIterations>>,
    ),
    mut mass_modified: EventWriter<MassModifiedEvent>,
) {
    let context = &mut *context;

    // Deal with sleeping first, because other changes may then wake-up the
    // rigid-body again.
    for (handle, sleeping) in changed_sleeping.iter() {
        if let Some(rb) = context.bodies.get_mut(handle) {
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
        if let Some(rb) = context.bodies.get_mut(handle) {
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
                    &handle,
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

        if let Some(rb) = context.bodies.get_mut(handle) {
            transform_changed = transform_changed.or_else(|| {
                Some(transform_changed_fn(
                    &handle,
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
                            .insert(handle, *global_transform);
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
                            .insert(handle, *global_transform);
                    }
                }
            }
        }
    }

    for (handle, velocity) in changed_velocities.iter() {
        if let Some(rb) = context.bodies.get_mut(handle) {
            rb.set_linvel(velocity.linvel, true);
            #[allow(clippy::useless_conversion)] // Need to convert if dim3 enabled
            rb.set_angvel(velocity.angvel, true);
        }
    }

    for (handle, mprops) in changed_additional_mass_props.iter() {
        if let Some(rb) = context.bodies.get_mut(handle) {
            match mprops {
                AdditionalMassProperties::MassProperties(mprops) => {
                    rb.set_additional_mass_properties(*mprops, true);
                }
                AdditionalMassProperties::Mass(mass) => {
                    rb.set_additional_mass(*mass, true);
                }
            }

            mass_modified.send(handle.into());
        }
    }

    for (handle, additional_solver_iters) in changed_additional_solver_iterations.iter() {
        if let Some(rb) = context.bodies.get_mut(handle) {
            rb.set_additional_solver_iterations(additional_solver_iters.0);
        }
    }

    for (handle, locked_axes) in changed_locked_axes.iter() {
        if let Some(rb) = context.bodies.get_mut(handle) {
            rb.set_locked_axes((*locked_axes).into(), true);
        }
    }

    for (handle, forces) in changed_forces.iter() {
        if let Some(rb) = context.bodies.get_mut(handle) {
            rb.reset_forces(true);
            rb.reset_torques(true);
            rb.add_force(forces.force, true);
            #[allow(clippy::useless_conversion)] // Need to convert if dim3 enabled
            rb.add_torque(forces.torque.into(), true);
        }
    }

    for (handle, mut impulses) in changed_impulses.iter_mut() {
        if let Some(rb) = context.bodies.get_mut(handle) {
            rb.apply_impulse(impulses.impulse, true);
            #[allow(clippy::useless_conversion)] // Need to convert if dim3 enabled
            rb.apply_torque_impulse(impulses.torque_impulse.into(), true);
            impulses.reset();
        }
    }

    for (handle, gravity_scale) in changed_gravity_scale.iter() {
        if let Some(rb) = context.bodies.get_mut(handle) {
            rb.set_gravity_scale(gravity_scale.0, true);
        }
    }

    for (handle, ccd) in changed_ccd.iter() {
        if let Some(rb) = context.bodies.get_mut(handle) {
            rb.enable_ccd(ccd.enabled);
        }
    }

    for (handle, dominance) in changed_dominance.iter() {
        if let Some(rb) = context.bodies.get_mut(handle) {
            rb.set_dominance_group(dominance.groups);
        }
    }

    for (handle, damping) in changed_damping.iter() {
        if let Some(rb) = context.bodies.get_mut(handle) {
            rb.set_linear_damping(damping.linear_damping);
            rb.set_angular_damping(damping.angular_damping);
        }
    }

    for (handle, _) in changed_disabled.iter() {
        if let Some(co) = context.bodies.get_mut(handle) {
            co.set_enabled(false);
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
    // We can’t use RigidBodyHandle yet because its creation command hasn’t been
    // executed yet.
    mut init_impulses: Query<(Entity, &mut ExternalImpulse), Without<RigidBodyCreated>>,
) {
    let context = &mut *context;

    for (entity, mut impulse) in init_impulses.iter_mut() {
        let bodies = &mut context.bodies;
        if let Some(rb) = bodies.get_mut(entity) {
            // Make sure the mass-properties are computed.
            rb.recompute_mass_properties_from_colliders(&context.colliders);
            // Apply the impulse.
            rb.apply_impulse(impulse.impulse, false);
            rb.apply_torque_impulse(impulse.torque_impulse, false);

            impulse.reset();
        }
    }
}
