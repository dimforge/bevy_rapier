//! Systems responsible for interfacing our Bevy components with the Rapier physics engine.

use crate::dynamics::{
    RapierImpulseJointHandle, RapierMultibodyJointHandle, RapierRigidBodyHandle,
};
use crate::geometry::RapierColliderHandle;
use crate::plugin::RapierContext;
use crate::prelude::PhysicsWorld;
use bevy::prelude::*;

/// If an entity is turned into the child of something with a physics world, the child should become a part of that physics world
///
/// If this fails to happen, weirdness will ensue.
pub fn on_add_entity_with_parent(
    q_add_entity_without_parent: Query<(Entity, &Parent), Changed<Parent>>,
    q_parent: Query<&Parent>,
    q_physics_world: Query<&PhysicsWorld>,
    mut commands: Commands,
) {
    for (ent, parent) in &q_add_entity_without_parent {
        let mut parent = Some(parent.get());
        while let Some(parent_entity) = parent {
            if let Ok(pw) = q_physics_world.get(parent_entity) {
                commands.entity(ent).insert(*pw);
                remove_old_physics(ent, &mut commands);
                break;
            }

            parent = q_parent.get(parent_entity).ok().map(|x| x.get());
        }
    }
}

/// Flags the entity to have its old physics removed
fn remove_old_physics(entity: Entity, commands: &mut Commands) {
    commands
        .entity(entity)
        .remove::<RapierColliderHandle>()
        .remove::<RapierRigidBodyHandle>()
        .remove::<RapierMultibodyJointHandle>()
        .remove::<RapierImpulseJointHandle>();
}

/// Flags the entity to have its physics updated to reflect new world
///
/// Also recursively bubbles down world changes to children & flags them to apply any needed physics changes
pub fn on_change_world(
    q_changed_worlds: Query<(Entity, &PhysicsWorld), Changed<PhysicsWorld>>,
    q_children: Query<&Children>,
    q_physics_world: Query<&PhysicsWorld>,
    context: Res<RapierContext>,
    mut commands: Commands,
) {
    for (entity, new_physics_world) in &q_changed_worlds {
        // Ensure the world actually changed before removing them from the world
        if !context
            .get_world(new_physics_world.world_id)
            .map(|x| {
                // They are already apart of this world if any of these are true
                x.entity2impulse_joint.contains_key(&entity)
                    || x.entity2multibody_joint.contains_key(&entity)
                    || x.entity2collider.contains_key(&entity)
                    || x.entity2body.contains_key(&entity)
            })
            .unwrap_or(false)
        {
            remove_old_physics(entity, &mut commands);

            bubble_down_world_change(
                &mut commands,
                entity,
                &q_children,
                *new_physics_world,
                &q_physics_world,
            );
        }
    }
}

fn bubble_down_world_change(
    commands: &mut Commands,
    entity: Entity,
    q_children: &Query<&Children>,
    new_physics_world: PhysicsWorld,
    q_physics_world: &Query<&PhysicsWorld>,
) {
    let Ok(children) = q_children.get(entity) else {
        return;
    };

    children.iter().for_each(|&child| {
        if q_physics_world
            .get(child)
            .map(|x| *x == new_physics_world)
            .unwrap_or(false)
        {
            return;
        }

        remove_old_physics(child, commands);
        commands.entity(child).insert(new_physics_world);

        bubble_down_world_change(
            commands,
            child,
            q_children,
            new_physics_world,
            q_physics_world,
        );
    });
}
