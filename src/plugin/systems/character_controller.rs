use crate::control::CharacterCollision;
use crate::dynamics::RapierRigidBodyHandle;
use crate::geometry::RapierColliderHandle;
use crate::plugin::context::systemparams::RAPIER_CONTEXT_EXPECT_ERROR;
use crate::plugin::context::RapierContextEntityLink;
use crate::plugin::RapierConfiguration;
use crate::prelude::context::RapierContextColliders;
use crate::prelude::context::RapierContextSimulation;
use crate::prelude::context::RapierRigidBodySet;
use crate::prelude::KinematicCharacterController;
use crate::prelude::KinematicCharacterControllerOutput;
use crate::utils;
use bevy::prelude::*;
use rapier::math::Isometry;
use rapier::math::Real;
use rapier::parry::query::DefaultQueryDispatcher;
use rapier::pipeline::QueryFilter;

/// System responsible for applying the character controller translation to the underlying
/// collider.
pub fn update_character_controls(
    mut commands: Commands,
    config: Query<&RapierConfiguration>,
    mut context_access: Query<(
        &mut RapierContextSimulation,
        &mut RapierContextColliders,
        &mut RapierRigidBodySet,
    )>,
    mut character_controllers: Query<(
        Entity,
        &RapierContextEntityLink,
        &mut KinematicCharacterController,
        Option<&mut KinematicCharacterControllerOutput>,
        Option<&RapierColliderHandle>,
        Option<&RapierRigidBodyHandle>,
        Option<&GlobalTransform>,
    )>,
    mut transforms: Query<&mut Transform>,
) {
    for (
        entity,
        rapier_context_link,
        mut controller,
        output,
        collider_handle,
        body_handle,
        glob_transform,
    ) in character_controllers.iter_mut()
    {
        if let (Some(raw_controller), Some(translation)) =
            (controller.to_raw(), controller.translation)
        {
            let config = config
                .get(rapier_context_link.0)
                .expect("Could not get [`RapierConfiguration`]");
            let (mut context, mut context_colliders, mut rigidbody_set) = context_access
                .get_mut(rapier_context_link.0)
                .expect(RAPIER_CONTEXT_EXPECT_ERROR);

            let context = &mut *context;
            let rigidbody_set = &mut *rigidbody_set;
            let scaled_custom_shape =
                controller
                    .custom_shape
                    .as_ref()
                    .map(|(custom_shape, tra, rot)| {
                        // TODO: avoid the systematic scale somehow?
                        let mut scaled_shape = custom_shape.clone();
                        scaled_shape.set_scale(custom_shape.scale, config.scaled_shape_subdivision);

                        (scaled_shape, *tra, *rot)
                    });

            let parent_rigid_body = body_handle.map(|h| h.0).or_else(|| {
                collider_handle
                    .and_then(|h| context_colliders.colliders.get(h.0))
                    .and_then(|c| c.parent())
            });
            let entity_to_move = parent_rigid_body
                .and_then(|rb| rigidbody_set.rigid_body_entity(rb))
                .unwrap_or(entity);

            let (character_shape, character_pos) = if let Some((scaled_shape, tra, rot)) =
                &scaled_custom_shape
            {
                let mut shape_pos: Isometry<Real> = (*tra, *rot).into();

                if let Some(body) = body_handle.and_then(|h| rigidbody_set.bodies.get(h.0)) {
                    shape_pos = body.position() * shape_pos
                } else if let Some(gtransform) = glob_transform {
                    shape_pos = utils::transform_to_iso(&gtransform.compute_transform()) * shape_pos
                }

                (&*scaled_shape.raw, shape_pos)
            } else if let Some(collider) =
                collider_handle.and_then(|h| context_colliders.colliders.get(h.0))
            {
                (collider.shape(), *collider.position())
            } else {
                continue;
            };
            let character_shape = character_shape.clone_dyn();

            let exclude_collider = collider_handle.map(|h| h.0);

            let character_mass = controller
                .custom_mass
                .or_else(|| {
                    parent_rigid_body
                        .and_then(|h| rigidbody_set.bodies.get(h))
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

            let mut query_pipeline = context.broad_phase.as_query_pipeline_mut(
                &DefaultQueryDispatcher,
                &mut rigidbody_set.bodies,
                &mut context_colliders.colliders,
                filter,
            );
            // TODO: add filter for charactercontroller.

            let movement = raw_controller.move_shape(
                context.integration_parameters.dt,
                &query_pipeline.as_ref(),
                &*character_shape,
                &character_pos,
                translation.into(),
                |c| collisions.push(c),
            );

            if controller.apply_impulse_to_dynamic_bodies {
                raw_controller.solve_character_collision_impulses(
                    context.integration_parameters.dt,
                    &mut query_pipeline,
                    &*character_shape,
                    character_mass,
                    collisions.iter(),
                )
            }

            if let Ok(mut transform) = transforms.get_mut(entity_to_move) {
                // TODO: take the parent’s GlobalTransform rotation into account?
                transform.translation.x += movement.translation.x;
                transform.translation.y += movement.translation.y;
                #[cfg(feature = "dim3")]
                {
                    transform.translation.z += movement.translation.z;
                }
            }

            let converted_collisions = context
                .character_collisions_collector
                .iter()
                .filter_map(|c| CharacterCollision::from_raw(&context_colliders, c));

            if let Some(mut output) = output {
                output.desired_translation = controller.translation.unwrap();
                output.effective_translation = movement.translation.into();
                output.grounded = movement.grounded;
                output.collisions.clear();
                output.collisions.extend(converted_collisions);
                output.is_sliding_down_slope = movement.is_sliding_down_slope;
            } else {
                commands
                    .entity(entity)
                    .insert(KinematicCharacterControllerOutput {
                        desired_translation: controller.translation.unwrap(),
                        effective_translation: movement.translation.into(),
                        grounded: movement.grounded,
                        collisions: converted_collisions.collect(),
                        is_sliding_down_slope: movement.is_sliding_down_slope,
                    });
            }

            controller.translation = None;
        }
    }
}
