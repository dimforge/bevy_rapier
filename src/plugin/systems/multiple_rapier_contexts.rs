//! systems to support multiple worlds, and changes between them.

use crate::dynamics::{
    RapierImpulseJointHandle, RapierMultibodyJointHandle, RapierRigidBodyHandle,
};
use crate::geometry::RapierColliderHandle;
use crate::plugin::{RapierContext, RapierContextEntityLink};
use bevy::prelude::*;

/// If an entity is turned into the child of something with a physics world, the child should become a part of that physics world
///
/// If this fails to happen, weirdness will ensue.
pub fn on_add_entity_with_parent(
    q_add_entity_without_parent: Query<
        (Entity, &Parent),
        (With<RapierContextEntityLink>, Changed<Parent>),
    >,
    q_parent: Query<&Parent>,
    q_physics_world: Query<&RapierContextEntityLink>,
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
    q_changed_worlds: Query<
        (Entity, Ref<RapierContextEntityLink>),
        Changed<RapierContextEntityLink>,
    >,
    q_children: Query<&Children>,
    q_physics_world: Query<&RapierContextEntityLink>,
    q_context: Query<&RapierContext>,
    mut commands: Commands,
) {
    for (entity, new_physics_world) in &q_changed_worlds {
        let context = q_context.get(new_physics_world.0);
        if new_physics_world.is_added() {
            continue;
        }
        // Ensure the world actually changed before removing them from the world
        if !context
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
    new_physics_world: RapierContextEntityLink,
    q_physics_world: &Query<&RapierContextEntityLink>,
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

#[cfg(test)]
mod test {
    use crate::plugin::systems::tests::HeadlessRenderPlugin;
    use crate::plugin::{
        NoUserData, PhysicsSet, RapierContext, RapierContextEntityLink, RapierPhysicsPlugin,
    };
    use crate::prelude::{ActiveEvents, Collider, ContactForceEventThreshold, RigidBody, Sensor};
    use bevy::prelude::*;
    use bevy::time::{TimePlugin, TimeUpdateStrategy};
    use rapier::math::Real;

    #[test]
    pub fn multi_world_hierarchy_update() {
        let mut app = App::new();
        app.add_plugins((
            HeadlessRenderPlugin,
            TransformPlugin,
            TimePlugin,
            RapierPhysicsPlugin::<NoUserData>::default(),
        ))
        .add_systems(
            PostUpdate,
            setup_physics
                .run_if(run_once())
                .before(PhysicsSet::SyncBackend),
        );
        // Simulates 60 updates per seconds
        app.insert_resource(TimeUpdateStrategy::ManualDuration(
            std::time::Duration::from_secs_f32(1f32 / 60f32),
        ));
        app.update();
        // Verify all rapier entities have a `RapierContextEntityLink`.
        let mut world = app.world_mut();
        let mut query = world.query_filtered::<Entity, With<Marker<'R'>>>();
        for entity in query.iter(&world) {
            world
                .get::<RapierContextEntityLink>(entity)
                .unwrap_or_else(|| panic!("no link to rapier context entity from {entity}."));
        }
        // Verify link is correctly updated for children.
        let new_rapier_context = world.spawn(RapierContext::default()).id();
        // FIXME: We need to wait 1 frame when creating a world.
        // Ideally we should be able to order the systems so that we don't have to wait.
        app.update();
        let mut world = app.world_mut();
        let mut query = world.query_filtered::<&mut RapierContextEntityLink, With<Marker<'P'>>>();
        let mut link_parent = query.get_single_mut(&mut world).unwrap();
        link_parent.0 = new_rapier_context;
        app.update();
        let mut world = app.world_mut();
        let mut query = world.query_filtered::<&RapierContextEntityLink, With<Marker<'C'>>>();
        let link_child = query.get_single_mut(&mut world).unwrap();
        assert_eq!(link_child.0, new_rapier_context);
        return;

        #[derive(Component)]
        pub struct Marker<const MARKER: char>;

        #[cfg(feature = "dim3")]
        fn cuboid(hx: Real, hy: Real, hz: Real) -> Collider {
            Collider::cuboid(hx, hy, hz)
        }
        #[cfg(feature = "dim2")]
        fn cuboid(hx: Real, hy: Real, _hz: Real) -> Collider {
            Collider::cuboid(hx, hy)
        }
        pub fn setup_physics(mut commands: Commands) {
            commands.spawn((
                TransformBundle::from(Transform::from_xyz(0.0, -1.2, 0.0)),
                cuboid(4.0, 1.0, 1.0),
                Marker::<'R'>,
            ));

            commands.spawn((
                TransformBundle::from(Transform::from_xyz(0.0, 5.0, 0.0)),
                cuboid(4.0, 1.5, 1.0),
                Sensor,
                Marker::<'R'>,
            ));

            commands
                .spawn((
                    TransformBundle::from(Transform::from_xyz(0.0, 13.0, 0.0)),
                    RigidBody::Dynamic,
                    cuboid(0.5, 0.5, 0.5),
                    ActiveEvents::COLLISION_EVENTS,
                    ContactForceEventThreshold(30.0),
                    Marker::<'P'>,
                    Marker::<'R'>,
                ))
                .with_children(|child_builder| {
                    child_builder.spawn((
                        TransformBundle::from(Transform::from_xyz(0.0, -1.2, 0.0)),
                        cuboid(4.0, 1.0, 1.0),
                        Marker::<'C'>,
                        Marker::<'R'>,
                    ));
                });
        }
    }
}
