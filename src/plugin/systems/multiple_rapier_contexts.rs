//! systems to support multiple physics contexts, and changes between them.

use crate::plugin::context::RapierRigidBodySet;
use crate::plugin::context::{
    RapierContextColliders, RapierContextEntityLink, RapierContextJoints,
};
use crate::plugin::systems::remove_all_physics;
use crate::prelude::{MassModifiedMessage, RapierContextSimulation};
use bevy::prelude::*;

/// If an entity is turned into the child of something with a physics context link,
/// the child should become a part of that physics context
///
/// If this fails to happen, weirdness will ensue.
pub fn on_add_entity_with_parent(
    q_add_entity_without_parent: Query<
        (Entity, &ChildOf),
        (
            With<RapierContextEntityLink>,
            Or<(Changed<RapierContextEntityLink>, Changed<ChildOf>)>,
        ),
    >,
    q_child_of: Query<&ChildOf>,
    q_rapier_context_link: Query<&RapierContextEntityLink>,
    mut commands: Commands,
    mut context_writer: Query<(
        &mut RapierContextSimulation,
        &mut RapierContextColliders,
        &mut RapierContextJoints,
        &mut RapierRigidBodySet,
    )>,
    mut mass_modified: MessageWriter<MassModifiedMessage>,
) {
    for (ent, child_of) in &q_add_entity_without_parent {
        let mut parent = Some(child_of.parent());
        while let Some(parent_entity) = parent {
            if let Ok(context_entity_link) = q_rapier_context_link.get(parent_entity) {
                // Change rapier context link only if the existing link isn't the correct one.
                if q_rapier_context_link
                    .get(ent)
                    .map(|x| x != context_entity_link)
                    .unwrap_or(true)
                {
                    remove_all_physics(ent, &mut context_writer, &mut mass_modified);
                    commands.entity(ent).insert(*context_entity_link);
                }
                break;
            }
            parent = q_child_of.get(parent_entity).ok().map(|x| x.parent());
        }
    }
}

/// Reacts to modifications to [`RapierContextEntityLink`]
/// to move an entity's physics data from a context to another.
///
/// Also recursively bubbles down context changes to children & flags them to apply any needed physics changes
pub fn on_change_context(
    q_changed_contexts: Query<
        (Entity, Ref<RapierContextEntityLink>),
        Changed<RapierContextEntityLink>,
    >,
    q_children: Query<&Children>,
    q_physics_context: Query<&RapierContextEntityLink>,
    mut commands: Commands,
    mut context_writer: Query<(
        &mut RapierContextSimulation,
        &mut RapierContextColliders,
        &mut RapierContextJoints,
        &mut RapierRigidBodySet,
    )>,
    mut mass_modified: MessageWriter<MassModifiedMessage>,
) {
    for (entity, new_physics_context) in &q_changed_contexts {
        let context = context_writer.get(new_physics_context.0);
        // Ensure the context actually changed before removing them from the context
        if !context
            .map(|(_, colliders, joints, rigidbody_set)| {
                // They are already apart of this context if any of these are true
                colliders.entity2collider.contains_key(&entity)
                    || rigidbody_set.entity2body.contains_key(&entity)
                    || joints.entity2impulse_joint.contains_key(&entity)
                    || joints.entity2multibody_joint.contains_key(&entity)
            })
            .unwrap_or(false)
        {
            info!("Adding new context to top-level {entity:?}");
            remove_all_physics(entity, &mut context_writer, &mut mass_modified);
            bubble_down_context_change(
                &mut commands,
                entity,
                &q_children,
                *new_physics_context,
                &q_physics_context,
                &mut context_writer,
                &mut mass_modified,
            );
        }
    }
}

fn bubble_down_context_change(
    commands: &mut Commands,
    entity: Entity,
    q_children: &Query<&Children>,
    new_physics_context: RapierContextEntityLink,
    q_physics_context: &Query<&RapierContextEntityLink>,
    context_writer: &mut Query<(
        &mut RapierContextSimulation,
        &mut RapierContextColliders,
        &mut RapierContextJoints,
        &mut RapierRigidBodySet,
    )>,
    mass_modified: &mut MessageWriter<MassModifiedMessage>,
) {
    let Ok(children) = q_children.get(entity) else {
        return;
    };

    children.iter().for_each(|child| {
        if q_physics_context
            .get(child)
            .map(|x| *x == new_physics_context)
            .unwrap_or(false)
        {
            return;
        }

        info!("Adding new context to child {child:?}");
        remove_all_physics(child, context_writer, mass_modified);
        commands.entity(child).insert(new_physics_context);

        bubble_down_context_change(
            commands,
            child,
            q_children,
            new_physics_context,
            q_physics_context,
            context_writer,
            mass_modified,
        );
    });
}

#[cfg(test)]
mod test {
    use crate::plugin::{
        context::{RapierContextEntityLink, RapierContextSimulation},
        NoUserData, PhysicsSet, RapierPhysicsPlugin,
    };
    use crate::prelude::{ActiveEvents, Collider, ContactForceEventThreshold, RigidBody, Sensor};
    use bevy::prelude::*;
    use bevy::time::{TimePlugin, TimeUpdateStrategy};
    use rapier::math::Real;

    #[test]
    pub fn multi_context_hierarchy_update() {
        let mut app = App::new();
        app.add_plugins((
            TransformPlugin,
            TimePlugin,
            RapierPhysicsPlugin::<NoUserData>::default(),
        ))
        .add_systems(
            PostUpdate,
            setup_physics
                .run_if(run_once)
                .before(PhysicsSet::SyncBackend),
        );
        // Simulates 60 updates per seconds
        app.insert_resource(TimeUpdateStrategy::ManualDuration(
            std::time::Duration::from_secs_f32(1f32 / 60f32),
        ));
        app.finish();
        app.update();
        // Verify all rapier entities have a `RapierContextEntityLink`.
        let world = app.world_mut();
        let mut query = world.query_filtered::<Entity, With<Marker<'R'>>>();
        for entity in query.iter(world) {
            world
                .get::<RapierContextEntityLink>(entity)
                .unwrap_or_else(|| panic!("no link to rapier context entity from {entity}."));
        }
        // Verify link is correctly updated for children.
        let new_rapier_context = world.spawn((RapierContextSimulation::default(),)).id();
        // FIXME: We need to wait 1 frame when creating a context.
        // Ideally we should be able to order the systems so that we don't have to wait.
        app.update();
        let world = app.world_mut();
        let mut query = world.query_filtered::<&mut RapierContextEntityLink, With<Marker<'P'>>>();
        let mut link_parent = query.single_mut(world).unwrap();
        link_parent.0 = new_rapier_context;
        app.update();
        let world = app.world_mut();
        let mut query = world.query_filtered::<&RapierContextEntityLink, With<Marker<'C'>>>();
        let link_child = query.single_mut(world).unwrap();
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
                Transform::from_xyz(0.0, -1.2, 0.0),
                cuboid(4.0, 1.0, 1.0),
                Marker::<'R'>,
            ));

            commands.spawn((
                Transform::from_xyz(0.0, 5.0, 0.0),
                cuboid(4.0, 1.5, 1.0),
                Sensor,
                Marker::<'R'>,
            ));

            commands
                .spawn((
                    Transform::from_xyz(0.0, 13.0, 0.0),
                    RigidBody::Dynamic,
                    cuboid(0.5, 0.5, 0.5),
                    ActiveEvents::COLLISION_EVENTS,
                    ContactForceEventThreshold(30.0),
                    Marker::<'P'>,
                    Marker::<'R'>,
                ))
                .with_children(|child_builder| {
                    child_builder.spawn((
                        Transform::from_xyz(0.0, -1.2, 0.0),
                        cuboid(4.0, 1.0, 1.0),
                        Marker::<'C'>,
                        Marker::<'R'>,
                    ));
                });
        }
    }
}
