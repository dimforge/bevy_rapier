use crate::dynamics::RigidBodyHandle;
use crate::plugin::{RapierConfiguration, RapierContext};
use crate::prelude::{
    KinematicCharacterController, KinematicCharacterControllerOutput, RigidBodyCreated,
};
use crate::utils;
use bevy::prelude::*;
use rapier::prelude::*;

use crate::control::CharacterCollision;
#[cfg(feature = "dim2")]
use bevy::math::Vec3Swizzles;

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
        Option<&RigidBodyCreated>,
        Option<&GlobalTransform>,
    )>,
    mut transforms: Query<&mut Transform>,
) {
    let context = &mut *context;
    for (entity, mut controller, output, body_created, glob_transform) in
        character_controllers.iter_mut()
    {
        if let (Some(raw_controller), Some(translation)) =
            (controller.to_raw(), controller.translation)
        {
            let scaled_custom_shape =
                controller
                    .custom_shape
                    .as_ref()
                    .map(|(custom_shape, shape_pos)| {
                        // TODO: avoid the systematic scale somehow?
                        let mut scaled_shape = custom_shape.clone();
                        scaled_shape.set_scale(custom_shape.scale, config.scaled_shape_subdivision);

                        (scaled_shape, *shape_pos)
                    });

            let parent_rigid_body = body_created
                .map(|_| entity)
                .or_else(|| context.colliders.get(entity).and_then(|c| c.parent()));
            let entity_to_move = parent_rigid_body.unwrap_or(entity);

            let (character_shape, character_pos) = if let Some((scaled_shape, shape_pos)) =
                &scaled_custom_shape
            {
                let mut shape_pos = *shape_pos;

                if let Some(body) = body_created.and_then(|_| context.bodies.get(entity)) {
                    shape_pos = *body.position() * shape_pos
                } else if let Some(gtransform) = glob_transform {
                    shape_pos = utils::transform_to_iso(&gtransform.compute_transform()) * shape_pos
                }

                (&*scaled_shape.raw, shape_pos)
            } else if let Some(collider) = context.colliders.get(entity) {
                (collider.shape(), *collider.position())
            } else {
                continue;
            };

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
                groups: controller.filter_groups.map(|g| g.0),
                exclude_collider: None,
                exclude_rigid_body: None,
                predicate: None,
            };

            if let Some(parent) = parent_rigid_body {
                filter = filter.exclude_rigid_body(parent);
            } else {
                filter = filter.exclude_collider(entity)
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
                translation,
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

            if let Ok(mut transform) = transforms.get_mut(entity_to_move) {
                // TODO: take the parent’s GlobalTransform rotation into account?
                transform.translation.x += movement.translation.x;
                transform.translation.y += movement.translation.y;
                #[cfg(feature = "dim3")]
                {
                    transform.translation.z += movement.translation.z;
                }
            }

            let converted_collisions = context.character_collisions_collector.iter().copied();

            if let Some(mut output) = output {
                output.desired_translation = controller.translation.unwrap();
                output.effective_translation = movement.translation;
                output.grounded = movement.grounded;
                output.collisions.clear();
                output.collisions.extend(converted_collisions);
            } else {
                commands
                    .entity(entity)
                    .insert(KinematicCharacterControllerOutput {
                        desired_translation: controller.translation.unwrap(),
                        effective_translation: movement.translation,
                        grounded: movement.grounded,
                        collisions: converted_collisions.collect(),
                    });
            }

            controller.translation = None;
        }
    }
}
