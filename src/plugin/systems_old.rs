//! Systems responsible for interfacing our Bevy components with the Rapier physics engine.

use crate::control::CharacterCollision;
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
use crate::plugin::configuration::{SimulationToRenderTime, TimestepMode};
use crate::plugin::{RapierConfiguration, RapierContext};
use crate::prelude::{
    AdditionalSolverIterations, BevyPhysicsHooks, BevyPhysicsHooksAdapter, CollidingEntities,
    ContactSkin, KinematicCharacterController, KinematicCharacterControllerOutput,
    MassModifiedEvent, PhysicsWorld, RapierWorld, Real, RigidBodyDisabled, WorldId,
    DEFAULT_WORLD_ID,
};
use crate::utils;
use bevy::ecs::system::{StaticSystemParam, SystemParamItem};
use bevy::prelude::*;
use rapier::prelude::*;
use std::collections::HashMap;

use super::{find_item_and_world, get_world};

#[cfg(all(feature = "dim3", feature = "async-collider"))]
use {
    crate::prelude::{AsyncCollider, AsyncSceneCollider},
    bevy::scene::SceneInstance,
};

/// Components that will be updated after a physics step.
pub type RigidBodyWritebackComponents<'a> = (
    Entity,
    Option<&'a mut Transform>,
    Option<&'a mut TransformInterpolation>,
    Option<&'a mut Velocity>,
    Option<&'a mut Sleeping>,
    Option<&'a PhysicsWorld>,
    Option<&'a RigidBody>,
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
    (
        Option<&'a Damping>,
        Option<&'a RigidBodyDisabled>,
        Option<&'a PhysicsWorld>,
        Option<&'a AdditionalSolverIterations>,
    ),
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
    Option<&'a ContactSkin>,
    Option<&'a CollisionGroups>,
    Option<&'a SolverGroups>,
    Option<&'a ContactForceEventThreshold>,
    Option<&'a ColliderDisabled>,
);

// Changes the world something is in.
// This will also change the children of that entity to reflect the new world.
fn recursively_apply_world_update(
    children_query: &Query<&Children>,
    physics_world_query: &Query<(Entity, Ref<PhysicsWorld>)>,
    context: &RapierContext,
    entity: Entity,
    commands: &mut Commands,
    new_world: WorldId,
) {
    if let Ok((_, physics_world)) = physics_world_query.get(entity) {
        if physics_world.world_id != new_world {
            // Needs to insert instead of changing a mut physics_world because
            // the Ref<PhysicsWorld> requires physics_world to not be mut.
            commands.entity(entity).insert(PhysicsWorld {
                world_id: new_world,
            });
        }

        // This currently loops through every world to find it, which
        // isn't the most efficient but gets the job done.
        if context
            .get_world(new_world)
            .map(|world| world.entity2body.contains_key(&entity))
            .unwrap_or(false)
        {
            // The value of the component did not change, no need to bubble it down or remove it from the world
            return;
        }

        // This entity will be picked up by the "init_colliders" systems and added
        // to the correct world if it is missing these components.
        commands
            .entity(entity)
            .remove::<RapierColliderHandle>()
            .remove::<RapierRigidBodyHandle>()
            .remove::<RapierMultibodyJointHandle>()
            .remove::<RapierImpulseJointHandle>();
    } else {
        commands
            .entity(entity)
            .insert(PhysicsWorld {
                world_id: new_world,
            })
            .remove::<RapierColliderHandle>()
            .remove::<RapierRigidBodyHandle>()
            .remove::<RapierMultibodyJointHandle>()
            .remove::<RapierImpulseJointHandle>();
    }

    // Carries down world changes to children
    if let Ok(children) = children_query.get(entity) {
        for child in children.iter() {
            recursively_apply_world_update(
                children_query,
                physics_world_query,
                context,
                *child,
                commands,
                new_world,
            );
        }
    }
}

/// Whenever an entity has its PhysicsWorld component changed, this
/// system places it in the new world & removes it from the old.
///
/// This does NOT add the entity to the new world, only signals that
/// it needs changed. Later down the line, systems will pick up this
/// entity that needs added & do everything necessary to add it.
///
/// This system will carry this change down to the children of that entity.
pub fn apply_changing_worlds(
    mut commands: Commands,
    physics_world_query: Query<(Entity, Ref<PhysicsWorld>)>,
    children_query: Query<&Children>,
    context: Res<RapierContext>,
) {
    for (entity, physics_world) in physics_world_query.iter() {
        if physics_world.is_added() || physics_world.is_changed() {
            recursively_apply_world_update(
                &children_query,
                &physics_world_query,
                &context,
                entity,
                &mut commands,
                physics_world.world_id,
            );
        }
    }
}

/// System responsible for applying changes the user made to a rigid-body-related component.
pub fn apply_rigid_body_user_changes(
    mut context: ResMut<RapierContext>,
    config: Res<RapierConfiguration>,
    changed_rb_types: Query<
        (&RapierRigidBodyHandle, &RigidBody, Option<&PhysicsWorld>),
        Changed<RigidBody>,
    >,
    mut changed_transforms: Query<
        (
            &RapierRigidBodyHandle,
            &GlobalTransform,
            Option<&mut TransformInterpolation>,
            Option<&PhysicsWorld>,
        ),
        Changed<GlobalTransform>,
    >,
    changed_velocities: Query<
        (&RapierRigidBodyHandle, &Velocity, Option<&PhysicsWorld>),
        Changed<Velocity>,
    >,
    changed_additional_mass_props: Query<
        (
            Entity,
            &RapierRigidBodyHandle,
            &AdditionalMassProperties,
            Option<&PhysicsWorld>,
        ),
        Changed<AdditionalMassProperties>,
    >,
    changed_locked_axes: Query<
        (&RapierRigidBodyHandle, &LockedAxes, Option<&PhysicsWorld>),
        Changed<LockedAxes>,
    >,
    changed_forces: Query<
        (
            &RapierRigidBodyHandle,
            &ExternalForce,
            Option<&PhysicsWorld>,
        ),
        Changed<ExternalForce>,
    >,
    mut changed_impulses: Query<
        (
            &RapierRigidBodyHandle,
            &mut ExternalImpulse,
            Option<&PhysicsWorld>,
        ),
        Changed<ExternalImpulse>,
    >,
    changed_gravity_scale: Query<
        (&RapierRigidBodyHandle, &GravityScale, Option<&PhysicsWorld>),
        Changed<GravityScale>,
    >,
    changed_ccd: Query<(&RapierRigidBodyHandle, &Ccd, Option<&PhysicsWorld>), Changed<Ccd>>,
    changed_dominance: Query<
        (&RapierRigidBodyHandle, &Dominance, Option<&PhysicsWorld>),
        Changed<Dominance>,
    >,
    changed_sleeping: Query<
        (&RapierRigidBodyHandle, &Sleeping, Option<&PhysicsWorld>),
        Changed<Sleeping>,
    >,
    changed_damping: Query<
        (&RapierRigidBodyHandle, &Damping, Option<&PhysicsWorld>),
        Changed<Damping>,
    >,
    (changed_disabled, changed_additional_solver_iterations): (
        Query<
            (
                &RapierRigidBodyHandle,
                &RigidBodyDisabled,
                Option<&PhysicsWorld>,
            ),
            Changed<RigidBodyDisabled>,
        >,
        Query<
            (
                &RapierRigidBodyHandle,
                &AdditionalSolverIterations,
                Option<&PhysicsWorld>,
            ),
            Changed<AdditionalSolverIterations>,
        >,
    ),

    mut mass_modified: EventWriter<MassModifiedEvent>,
) {
    // Deal with sleeping first, because other changes may then wake-up the
    // rigid-body again.
    for (handle, sleeping, world_within) in changed_sleeping.iter() {
        let world = get_world(world_within, &mut context);

        if let Some(rb) = world.bodies.get_mut(handle.0) {
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
    for (handle, rb_type, world_within) in changed_rb_types.iter() {
        let world = get_world(world_within, &mut context);

        if let Some(rb) = world.bodies.get_mut(handle.0) {
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

    for (handle, global_transform, mut interpolation, world_within) in changed_transforms.iter_mut()
    {
        let world = get_world(world_within, &mut context);

        // Use an Option<bool> to avoid running the check twice.
        let mut transform_changed = None;

        if let Some(interpolation) = interpolation.as_deref_mut() {
            transform_changed = transform_changed.or_else(|| {
                Some(transform_changed_fn(
                    &handle.0,
                    global_transform,
                    &world.last_body_transform_set,
                ))
            });

            if transform_changed == Some(true) {
                // Reset the interpolation so we don’t overwrite
                // the user’s input.
                interpolation.start = None;
                interpolation.end = None;
            }
        }

        transform_changed = transform_changed.or_else(|| {
            Some(transform_changed_fn(
                &handle.0,
                global_transform,
                &world.last_body_transform_set,
            ))
        });

        if transform_changed == Some(true) {
            if let Some(rb) = world.bodies.get_mut(handle.0) {
                match rb.body_type() {
                    RigidBodyType::KinematicPositionBased => {
                        if transform_changed == Some(true) {
                            rb.set_next_kinematic_position(utils::transform_to_iso(
                                &global_transform.compute_transform(),
                                world.physics_scale,
                            ));
                            world
                                .last_body_transform_set
                                .insert(handle.0, *global_transform);
                        }
                    }
                    _ => {
                        rb.set_position(
                            utils::transform_to_iso(
                                &global_transform.compute_transform(),
                                world.physics_scale,
                            ),
                            true,
                        );
                        world
                            .last_body_transform_set
                            .insert(handle.0, *global_transform);
                    }
                }
            }
        }
    }

    for (handle, velocity, world_within) in changed_velocities.iter() {
        let world = get_world(world_within, &mut context);

        // get here instead of get_mut to avoid change detection if it doesn't need to be changed
        if let Some(rb) = world.bodies.get(handle.0) {
            let new_linvel = (velocity.linvel / world.physics_scale).into();
            #[allow(clippy::useless_conversion)] // Need to convert if dim3 enabled
            let new_angvel = velocity.angvel.into();

            #[cfg(feature = "dim3")]
            let cur_angvel = *rb.angvel();
            #[cfg(feature = "dim2")]
            let cur_angvel = rb.angvel();

            let is_different = *rb.linvel() != new_linvel || cur_angvel != new_angvel;

            if is_different {
                let rb = world
                    .bodies
                    .get_mut(handle.0)
                    .expect("Verified to exist in above world.bodies.get");

                rb.set_linvel(new_linvel, true);
                rb.set_angvel(new_angvel, true);
            }
        }
    }

    for (entity, handle, mprops, world_within) in changed_additional_mass_props.iter() {
        let world = get_world(world_within, &mut context);

        if let Some(rb) = world.bodies.get_mut(handle.0) {
            match mprops {
                AdditionalMassProperties::MassProperties(mprops) => {
                    rb.set_additional_mass_properties(
                        mprops.into_rapier(world.physics_scale),
                        true,
                    );
                }
                AdditionalMassProperties::Mass(mass) => {
                    rb.set_additional_mass(*mass, true);
                }
            }

            mass_modified.send(entity.into());
        }
    }

    for (handle, additional_solver_iters, world_within) in
        changed_additional_solver_iterations.iter()
    {
        let world = get_world(world_within, &mut context);

        if let Some(rb) = world.bodies.get_mut(handle.0) {
            rb.set_additional_solver_iterations(additional_solver_iters.0);
        }
    }

    for (handle, locked_axes, world_within) in changed_locked_axes.iter() {
        let world = get_world(world_within, &mut context);

        if let Some(rb) = world.bodies.get_mut(handle.0) {
            rb.set_locked_axes((*locked_axes).into(), true);
        }
    }

    for (handle, forces, world_within) in changed_forces.iter() {
        let world = get_world(world_within, &mut context);

        if let Some(rb) = world.bodies.get_mut(handle.0) {
            rb.reset_forces(true);
            rb.reset_torques(true);
            rb.add_force((forces.force / world.physics_scale).into(), true);
            #[allow(clippy::useless_conversion)] // Need to convert if dim3 enabled
            rb.add_torque(forces.torque.into(), true);
        }
    }

    for (handle, mut impulses, world_within) in changed_impulses.iter_mut() {
        let world = get_world(world_within, &mut context);

        if let Some(rb) = world.bodies.get_mut(handle.0) {
            rb.apply_impulse((impulses.impulse / world.physics_scale).into(), true);
            #[allow(clippy::useless_conversion)] // Need to convert if dim3 enabled
            rb.apply_torque_impulse(impulses.torque_impulse.into(), true);
            impulses.reset();
        }
    }

    for (handle, gravity_scale, world_within) in changed_gravity_scale.iter() {
        let world = get_world(world_within, &mut context);

        if let Some(rb) = world.bodies.get_mut(handle.0) {
            rb.set_gravity_scale(gravity_scale.0, true);
        }
    }

    for (handle, ccd, world_within) in changed_ccd.iter() {
        let world = get_world(world_within, &mut context);

        if let Some(rb) = world.bodies.get_mut(handle.0) {
            rb.enable_ccd(ccd.enabled);
        }
    }

    for (handle, dominance, world_within) in changed_dominance.iter() {
        let world = get_world(world_within, &mut context);

        if let Some(rb) = world.bodies.get_mut(handle.0) {
            rb.set_dominance_group(dominance.groups);
        }
    }

    for (handle, damping, world_within) in changed_damping.iter() {
        let world = get_world(world_within, &mut context);

        if let Some(rb) = world.bodies.get_mut(handle.0) {
            rb.set_linear_damping(damping.linear_damping);
            rb.set_angular_damping(damping.angular_damping);
        }
    }

    for (handle, _, world_within) in changed_disabled.iter() {
        let world = get_world(world_within, &mut context);

        if let Some(co) = world.bodies.get_mut(handle.0) {
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
    top_entities: Query<Entity, Without<Parent>>,
    mut writeback: Query<RigidBodyWritebackComponents, Without<RigidBodyDisabled>>,
    children_query: Query<&Children>,
) {
    for entity in top_entities.iter() {
        let (transform, delta_transform, velocity, world_offset) = if let Ok((
            entity,
            transform,
            mut interpolation,
            mut velocity,
            mut sleeping,
            world_within,
            _,
        )) = writeback.get_mut(entity)
        {
            let mut my_new_global_transform = Transform::IDENTITY;
            let mut parent_delta = Transform::IDENTITY;
            let mut my_velocity = Velocity::default();
            let mut world_offset = Vec3::ZERO;

            let world = get_world(world_within, &mut context);

            // TODO: do this the other way round: iterate through Rapier’s RigidBodySet on the active bodies,
            // and update the components accordingly. That way, we don’t have to iterate through the entities that weren’t changed
            // by physics (for example because they are sleeping).
            if let Some(handle) = world.entity2body.get(&entity).copied() {
                if let Some(rb) = world.bodies.get(handle) {
                    let mut interpolated_pos =
                        utils::iso_to_transform(rb.position(), world.physics_scale);

                    if let TimestepMode::Interpolated { dt, .. } = config.timestep_mode {
                        if let Some(interpolation) = interpolation.as_deref_mut() {
                            if interpolation.end.is_none() {
                                interpolation.end = Some(*rb.position());
                            }

                            if let Some(interpolated) =
                                interpolation.lerp_slerp((dt + sim_to_render_time.diff) / dt)
                            {
                                interpolated_pos =
                                    utils::iso_to_transform(&interpolated, world.physics_scale);
                            }
                        }
                    }

                    if let Some(mut transform) = transform {
                        // In 2D, preserve the transform `z` component that may have been set by the user
                        #[cfg(feature = "dim2")]
                        {
                            interpolated_pos.translation.z = transform.translation.z;
                        }

                        world_offset = transform.translation;

                        // let (cur_inv_scale, cur_inv_rotation, cur_inv_translation) = transform
                        //     .compute_affine()
                        //     .inverse()
                        //     .to_scale_rotation_translation();

                        parent_delta = Transform {
                            translation: interpolated_pos.translation - transform.translation,
                            rotation: interpolated_pos.rotation * transform.rotation.inverse(),
                            scale: transform.scale,
                        };

                        let com = rb.center_of_mass();

                        #[cfg(feature = "dim3")]
                        let com = Vec3::new(
                            com.x - rb.translation().x,
                            com.y - rb.translation().y,
                            com.z - rb.translation().z,
                        ) / world.physics_scale;
                        #[cfg(feature = "dim2")]
                        let com =
                            Vec3::new(com.x - rb.translation().x, com.y - rb.translation().y, 0.0)
                                / world.physics_scale;

                        let com_diff = com - parent_delta.rotation.mul_vec3(com);
                        parent_delta.translation -= com_diff;

                        if transform.rotation != interpolated_pos.rotation
                            || transform.translation != interpolated_pos.translation
                        {
                            // NOTE: we write the new value only if there was an
                            //       actual change, in order to not trigger bevy’s
                            //       change tracking when the values didn’t change.
                            transform.rotation = interpolated_pos.rotation;
                            transform.translation = interpolated_pos.translation;
                        }

                        my_new_global_transform = interpolated_pos;

                        world
                            .last_body_transform_set
                            .insert(handle, GlobalTransform::from(interpolated_pos));
                    }

                    if let Some(velocity) = &mut velocity {
                        my_velocity = **velocity;

                        let new_vel = Velocity {
                            linvel: (rb.linvel() * world.physics_scale).into(),
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

            (
                my_new_global_transform,
                parent_delta,
                my_velocity,
                world_offset,
            )
        } else {
            (
                Transform::IDENTITY,
                Transform::IDENTITY,
                Velocity::default(),
                Vec3::ZERO,
            )
        };

        recurse_child_transforms(
            context.as_mut(),
            &config,
            &sim_to_render_time,
            &mut writeback,
            transform,
            delta_transform,
            velocity,
            &children_query,
            entity,
            world_offset,
        );
    }
}

fn recurse_child_transforms(
    context: &mut RapierContext,
    config: &RapierConfiguration,
    sim_to_render_time: &SimulationToRenderTime,
    writeback: &mut Query<RigidBodyWritebackComponents, Without<RigidBodyDisabled>>,
    parent_global_transform: Transform,
    parent_delta: Transform,
    parent_velocity: Velocity,
    children_query: &Query<&Children>,
    parent_entity: Entity,
    world_offset: Vec3,
) {
    let Ok(children) = children_query.get(parent_entity) else {
        return;
    };

    for child in children.iter().copied() {
        let mut world_offset = world_offset;

        let (transform, delta_transform, velocity) = if let Ok((
            entity,
            transform,
            mut interpolation,
            mut velocity,
            mut sleeping,
            world_within,
            rb_type,
        )) = writeback.get_mut(child)
        {
            let mut my_new_global_transform = parent_global_transform;
            let mut delta_transform = parent_delta;
            let mut my_velocity = parent_velocity;

            let world = get_world(world_within, context);

            // TODO: do this the other way round: iterate through Rapier’s RigidBodySet on the active bodies,
            // and update the components accordingly. That way, we don’t have to iterate through the entities that weren’t changed
            // by physics (for example because they are sleeping).
            if let Some(handle) = world.entity2body.get(&entity).copied() {
                if let Some(rb) = world.bodies.get_mut(handle) {
                    let mut interpolated_pos =
                        utils::iso_to_transform(rb.position(), world.physics_scale);

                    if let TimestepMode::Interpolated { dt, .. } = config.timestep_mode {
                        if let Some(interpolation) = interpolation.as_deref_mut() {
                            if interpolation.end.is_none() {
                                interpolation.end = Some(*rb.position());
                            }

                            if let Some(interpolated) =
                                interpolation.lerp_slerp((dt + sim_to_render_time.diff) / dt)
                            {
                                interpolated_pos =
                                    utils::iso_to_transform(&interpolated, world.physics_scale);
                            }
                        }
                    }

                    if let Some(mut transform) = transform {
                        // We need to compute the new local transform such that:
                        // curr_parent_global_transform * new_transform * parent_delta_pos = interpolated_pos
                        // new_transform = curr_parent_global_transform.inverse() * interpolated_pos

                        let inverse_parent_rotation = parent_global_transform.rotation.inverse();

                        interpolated_pos.translation -= world_offset;

                        let new_rotation = Quat::IDENTITY; //inverse_parent_rotation * interpolated_pos.rotation;

                        // has to be mut in 2d mode
                        #[allow(unused_mut)]
                        let mut new_translation;

                        let translation_offset =
                            if rb_type.copied().unwrap_or(RigidBody::Fixed) == RigidBody::Dynamic {
                                // The parent's velocity will have already moved them
                                parent_delta.translation
                            } else {
                                Vec3::ZERO
                            };

                        let rotated_interpolation = inverse_parent_rotation
                            * (parent_delta.rotation
                                * (interpolated_pos.translation - translation_offset));

                        new_translation = rotated_interpolation;

                        // In 2D, preserve the transform `z` component that may have been set by the user
                        #[cfg(feature = "dim2")]
                        {
                            new_translation.z = transform.translation.z;
                        }

                        let old_transform = *transform;

                        if transform.rotation != new_rotation
                            || transform.translation != new_translation
                        {
                            // NOTE: we write the new value only if there was an
                            //       actual change, in order to not trigger bevy’s
                            //       change tracking when the values didn’t change.
                            transform.rotation = new_rotation;
                            transform.translation = new_translation;
                        }

                        let inv_old_transform = Transform {
                            scale: old_transform.scale,
                            rotation: old_transform.rotation.inverse(),
                            translation: -old_transform.translation,
                        };

                        delta_transform = transform.mul_transform(inv_old_transform);
                        // .mul_transform(parent_delta);

                        // NOTE: we need to compute the result of the next transform propagation
                        //       to make sure that our change detection for transforms is exact
                        //       despite rounding errors.

                        my_new_global_transform = parent_global_transform.mul_transform(*transform);
                        world_offset = my_new_global_transform.translation;

                        world
                            .last_body_transform_set
                            .insert(handle, GlobalTransform::from(my_new_global_transform));

                        rb.set_position(
                            utils::transform_to_iso(&my_new_global_transform, world.physics_scale),
                            false,
                        );

                        // rb.set_rotation(
                        //     Rotation::from_quaternion(Quaternion::new(
                        //         my_new_global_transform.rotation.w,
                        //         my_new_global_transform.rotation.x,
                        //         my_new_global_transform.rotation.y,
                        //         my_new_global_transform.rotation.z,
                        //     )),
                        //     false,
                        // );
                    }

                    if let Some(velocity) = &mut velocity {
                        let old_linvel = *rb.linvel();

                        my_velocity.linvel = (old_linvel * world.physics_scale).into();

                        rb.set_linvel((parent_velocity.linvel / world.physics_scale).into(), false);
                        rb.set_linvel(old_linvel - rb.linvel(), false);

                        let new_vel = Velocity {
                            linvel: (rb.linvel() * world.physics_scale).into(),
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

            (my_new_global_transform, delta_transform, my_velocity)
        } else {
            (parent_global_transform, parent_delta, parent_velocity)
        };

        recurse_child_transforms(
            context,
            config,
            sim_to_render_time,
            writeback,
            transform,
            delta_transform,
            velocity,
            children_query,
            child,
            world_offset,
        );
    }
}

/// Syncs up child velocities with their parents in the physics simulation.
/// This is done to avoid child components getting hit by their parent and rapier
/// assuming the child is hit by the full velocity of the parent instead of `parent vel - child vel`.
///
/// This will not change the bevy component's velocity.
pub fn sync_vel(
    top_ents: Query<Entity, Without<Parent>>,
    vel_query: Query<&Velocity>,
    query: Query<(&RapierRigidBodyHandle, Option<&PhysicsWorld>)>,
    children_query: Query<&Children>,
    mut context: ResMut<RapierContext>,
) {
    for ent in top_ents.iter() {
        let vel = if let Ok(velocity) = vel_query.get(ent) {
            *velocity
        } else {
            Velocity::default()
        };

        if let Ok(children) = children_query.get(ent) {
            for child in children.iter().copied() {
                sync_velocity_recursively(child, &query, &children_query, vel, &mut context);
            }
        }
    }
}

fn sync_velocity_recursively(
    ent: Entity,
    query: &Query<(&RapierRigidBodyHandle, Option<&PhysicsWorld>)>,
    children_query: &Query<&Children>,
    parent_vel: Velocity,
    context: &mut RapierContext,
) {
    let vel = if let Ok((handle, world_within)) = query.get(ent) {
        let world = get_world(world_within, context);
        if let Some(rb) = world.bodies.get_mut(handle.0) {
            #[cfg(feature = "dim3")]
            let old_linvel = Vec3::from(*rb.linvel());
            #[cfg(feature = "dim2")]
            let old_linvel = Vec2::from(*rb.linvel());

            rb.set_linvel(
                (old_linvel + (parent_vel.linvel / world.physics_scale)).into(),
                false,
            );

            Velocity {
                linvel: (rb.linvel() * world.physics_scale).into(),
                #[cfg(feature = "dim3")]
                angvel: (*rb.angvel()).into(),
                #[cfg(feature = "dim2")]
                angvel: rb.angvel(),
            }
        } else {
            parent_vel
        }
    } else {
        parent_vel
    };

    if let Ok(children) = children_query.get(ent) {
        for child in children.iter().copied() {
            sync_velocity_recursively(child, query, children_query, vel, context);
        }
    }
}

/// System responsible for advancing the physics simulation, and updating the internal state
/// for scene queries.
#[allow(clippy::too_many_arguments)]
pub fn step_simulation<Hooks>(
    mut context: ResMut<RapierContext>,
    config: Res<RapierConfiguration>,
    hooks: StaticSystemParam<Hooks>,
    time: Res<Time>,
    mut sim_to_render_time: ResMut<SimulationToRenderTime>,
    mut collision_event_writer: EventWriter<CollisionEvent>,
    mut contact_force_event_writer: EventWriter<ContactForceEvent>,
    mut interpolation_query: Query<(&RapierRigidBodyHandle, &mut TransformInterpolation)>,
) where
    Hooks: 'static + BevyPhysicsHooks,
    for<'w, 's> SystemParamItem<'w, 's, Hooks>: BevyPhysicsHooks,
{
    let hooks_adapter = BevyPhysicsHooksAdapter::new(hooks.into_inner());

    for (_, world) in context.worlds.iter_mut() {
        if config.physics_pipeline_active {
            world.step_simulation(
                config.gravity,
                config.timestep_mode,
                true,
                &hooks_adapter,
                &time,
                &mut sim_to_render_time,
                &mut Some(&mut interpolation_query),
            );

            world.deleted_colliders.clear();

            world.send_bevy_events(&mut collision_event_writer, &mut contact_force_event_writer);
        } else {
            world.propagate_modified_body_positions_to_colliders();
        }

        if config.query_pipeline_active {
            world.update_query_pipeline();
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
        ccd,
        dominance,
        sleep,
        (damping, disabled, world_within, additional_solver_iters),
    ) in rigid_bodies.iter()
    {
        let world = get_world(world_within, &mut context);

        let physics_scale = world.physics_scale;

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

        let handle = world.bodies.insert(rb);
        commands
            .entity(entity)
            .insert(RapierRigidBodyHandle(handle));

        world.entity2body.insert(entity, handle);

        if let Some(transform) = transform {
            world.last_body_transform_set.insert(handle, *transform);
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
    mut init_impulses: Query<
        (Entity, &mut ExternalImpulse, Option<&PhysicsWorld>),
        Without<RapierRigidBodyHandle>,
    >,
) {
    for (entity, mut impulse, world_within) in init_impulses.iter_mut() {
        let world = get_world(world_within, &mut context);

        let bodies = &mut world.bodies;
        if let Some(rb) = world
            .entity2body
            .get(&entity)
            .and_then(|h| bodies.get_mut(*h))
        {
            // Make sure the mass-properties are computed.
            rb.recompute_mass_properties_from_colliders(&world.colliders);
            // Apply the impulse.
            rb.apply_impulse((impulse.impulse / world.physics_scale).into(), false);

            #[allow(clippy::useless_conversion)] // Need to convert if dim3 enabled
            rb.apply_torque_impulse(impulse.torque_impulse.into(), false);

            impulse.reset();
        }
    }
}

#[cfg(test)]
mod tests {
    #[cfg(all(feature = "dim3", feature = "async-collider"))]
    use bevy::prelude::Cuboid;
    use bevy::{
        render::{
            settings::{RenderCreation, WgpuSettings},
            RenderPlugin,
        },
        scene::ScenePlugin,
        time::TimePlugin,
    };
    use std::f32::consts::PI;

    use super::*;
    use crate::plugin::{NoUserData, RapierPhysicsPlugin};

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

            app.update();

            let child_transform = app.world.entity(child).get::<GlobalTransform>().unwrap();
            let context = app.world.resource::<RapierContext>();
            let world = context
                .get_world(DEFAULT_WORLD_ID)
                .expect("The default world should exist.");

            let child_handle = world.entity2body[&child];
            let child_body = world.bodies.get(child_handle).unwrap();
            let body_transform =
                utils::iso_to_transform(child_body.position(), world.physics_scale);
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

            app.update();

            let child_transform = app
                .world
                .entity(child)
                .get::<GlobalTransform>()
                .unwrap()
                .compute_transform();
            let context = app.world.resource::<RapierContext>();
            let world = context
                .get_world(DEFAULT_WORLD_ID)
                .expect("The default world should exist.");

            let parent_handle = world.entity2body[&parent];
            let parent_body = world.bodies.get(parent_handle).unwrap();
            let child_collider_handle = parent_body.colliders()[0];
            let child_collider = world.colliders.get(child_collider_handle).unwrap();
            let body_transform =
                utils::iso_to_transform(child_collider.position(), world.physics_scale);
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
