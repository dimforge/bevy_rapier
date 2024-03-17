use crate::dynamics::{
    ReadMassProperties, RigidBody, SleepState, TransformInterpolation, Velocity,
};
use crate::plugin::configuration::{SimulationToRenderTime, TimestepMode};
use crate::plugin::{RapierConfiguration, RapierContext};
use crate::prelude::{MassModifiedEvent, RigidBodyDisabled};
use crate::utils;
use bevy::prelude::*;

#[cfg(feature = "dim2")]
use bevy::math::Vec3Swizzles;

/// Components that will be updated after a physics step.
pub type RigidBodyWritebackComponents<'a> = (
    Entity,
    Option<&'a Parent>,
    Option<&'a mut Transform>,
    Option<&'a mut TransformInterpolation>,
    Option<&'a mut Velocity>,
    Option<&'a mut SleepState>,
);

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
        for (entity, parent, transform, mut interpolation, mut velocity, mut sleeping) in
            writeback.iter_mut()
        {
            // TODO: do this the other way round: iterate through Rapier’s RigidBodySet on the active bodies,
            // and update the components accordingly. That way, we don’t have to iterate through the entities that weren’t changed
            // by physics (for example because they are sleeping).
            if let Some(rb) = context.bodies.get(entity) {
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
                            .insert(entity, new_global_transform);
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
                            .insert(entity, GlobalTransform::from(interpolated_pos));
                    }
                }

                if let Some(velocity) = &mut velocity {
                    let new_vel = Velocity {
                        linvel: *rb.linvel(),
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

/// System responsible for writing updated mass properties back into the [`ReadMassProperties`] component.
pub fn writeback_mass_properties(
    mut context: ResMut<RapierContext>,
    config: Res<RapierConfiguration>,
    mut mass_props: Query<&mut ReadMassProperties>,
    mut mass_modified: EventReader<MassModifiedEvent>,
) {
    let context = &mut *context;

    if config.physics_pipeline_active {
        for event in mass_modified.read() {
            if let Some(rb) = context.bodies.get(event.0) {
                if let Ok(mut mass_props) = mass_props.get_mut(event.0) {
                    let new_mass_props = rb.mass_properties().local_mprops;

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
