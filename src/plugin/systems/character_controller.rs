use crate::control::CharacterCollision;
use crate::dynamics::RapierRigidBodyHandle;
use crate::geometry::RapierColliderHandle;
use crate::plugin::get_world;
use crate::plugin::RapierConfiguration;
use crate::plugin::RapierContext;
use crate::prelude::KinematicCharacterController;
use crate::prelude::KinematicCharacterControllerOutput;
use crate::prelude::PhysicsWorld;
use crate::utils;
use bevy::prelude::*;
use rapier::math::Isometry;
use rapier::math::Real;
use rapier::pipeline::QueryFilter;

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
        Option<&PhysicsWorld>,
    )>,
    mut transforms: Query<&mut Transform>,
) {
    for (
        entity,
        mut controller,
        output,
        collider_handle,
        body_handle,
        glob_transform,
        world_within,
    ) in character_controllers.iter_mut()
    {
        let world = get_world(world_within, &mut context);

        if let (Some(raw_controller), Some(translation)) =
            (controller.to_raw(), controller.translation)
        {
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
                    .and_then(|h| world.colliders.get(h.0))
                    .and_then(|c| c.parent())
            });
            let entity_to_move = parent_rigid_body
                .and_then(|rb| world.rigid_body_entity(rb))
                .unwrap_or(entity);

            let (character_shape, character_pos) = if let Some((scaled_shape, tra, rot)) =
                &scaled_custom_shape
            {
                let mut shape_pos: Isometry<Real> = (*tra, *rot).into();

                if let Some(body) = body_handle.and_then(|h| world.bodies.get(h.0)) {
                    shape_pos = body.position() * shape_pos
                } else if let Some(gtransform) = glob_transform {
                    shape_pos = utils::transform_to_iso(&gtransform.compute_transform()) * shape_pos
                }

                (&*scaled_shape.raw, shape_pos)
            } else if let Some(collider) = collider_handle.and_then(|h| world.colliders.get(h.0)) {
                (collider.shape(), *collider.position())
            } else {
                continue;
            };

            let exclude_collider = collider_handle.map(|h| h.0);

            let character_mass = controller
                .custom_mass
                .or_else(|| {
                    parent_rigid_body
                        .and_then(|h| world.bodies.get(h))
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

            let collisions = &mut world.character_collisions_collector;
            collisions.clear();

            let movement = raw_controller.move_shape(
                world.integration_parameters.dt,
                &world.bodies,
                &world.colliders,
                &world.query_pipeline,
                character_shape,
                &character_pos,
                translation.into(),
                filter,
                |c| collisions.push(c),
            );

            if controller.apply_impulse_to_dynamic_bodies {
                for collision in collisions.iter() {
                    raw_controller.solve_character_collision_impulses(
                        world.integration_parameters.dt,
                        &mut world.bodies,
                        &world.colliders,
                        &world.query_pipeline,
                        character_shape,
                        character_mass,
                        collision,
                        filter,
                    )
                }
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

            let converted_collisions = world
                .character_collisions_collector
                .iter()
                .filter_map(|c| CharacterCollision::from_raw(world, c));

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
