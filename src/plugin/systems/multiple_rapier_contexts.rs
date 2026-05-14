//! systems to support multiple physics contexts, and changes between them.

use crate::dynamics::{
    RapierImpulseJointHandle, RapierMultibodyJointHandle, RapierRigidBodyHandle,
};
use crate::geometry::RapierColliderHandle;
use crate::plugin::context::RapierRigidBodySet;
use crate::plugin::context::{
    RapierContextColliders, RapierContextEntityLink, RapierContextJoints,
};
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
    q_physics_world: Query<&RapierContextEntityLink>,
    mut commands: Commands,
) {
    for (ent, child_of) in &q_add_entity_without_parent {
        let mut parent = Some(child_of.parent());
        while let Some(parent_entity) = parent {
            if let Ok(pw) = q_physics_world.get(parent_entity) {
                // Change rapier context link only if the existing link isn't the correct one.
                if q_physics_world.get(ent).map(|x| x != pw).unwrap_or(true) {
                    remove_old_physics(ent, &mut commands);
                    commands.entity(ent).insert(*pw);
                }
                break;
            }
            parent = q_child_of.get(parent_entity).ok().map(|x| x.parent());
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
    q_context: Query<(
        &RapierContextColliders,
        &RapierContextJoints,
        &RapierRigidBodySet,
    )>,
    mut commands: Commands,
) {
    for (entity, new_physics_context) in &q_changed_contexts {
        let context = q_context.get(new_physics_context.0);
        // Ensure the context actually changed before removing them from the context
        if !context
            .map(|(colliders, joints, rigidbody_set)| {
                // They are already apart of this context if any of these are true
                colliders.entity2collider.contains_key(&entity)
                    || rigidbody_set.entity2body.contains_key(&entity)
                    || joints.entity2impulse_joint.contains_key(&entity)
                    || joints.entity2multibody_joint.contains_key(&entity)
            })
            .unwrap_or(false)
        {
            remove_old_physics(entity, &mut commands);
            bubble_down_context_change(
                &mut commands,
                entity,
                &q_children,
                *new_physics_context,
                &q_physics_context,
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

        remove_old_physics(child, commands);
        commands.entity(child).insert(new_physics_context);

        bubble_down_context_change(
            commands,
            child,
            q_children,
            new_physics_context,
            q_physics_context,
        );
    });
}

#[cfg(test)]
mod test {
    use crate::plugin::{
        context::{
            DefaultRapierContext, RapierContextEntityLink, RapierContextSimulation,
            RapierRigidBodySet,
        },
        NoUserData, PhysicsSet, RapierPhysicsPlugin,
    };
    use crate::prelude::{
        ActiveEvents, Collider, ContactForceEventThreshold, RapierColliderHandle,
        RapierRigidBodyHandle, RigidBody, Sensor,
    };
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

    /// Regression test for `sync_removals` deleting a freshly-reinserted body when an entity is
    /// migrated between Rapier contexts in `.in_fixed_schedule()` mode.
    ///
    /// When the schedule is not `PostUpdate`, `RapierPhysicsPlugin` registers `sync_removals` in
    /// both the configured schedule's `SyncBackend` chain *and* in `PostUpdate` as a safety net
    /// for events that would otherwise be missed. Each registration owns an independent
    /// `RemovedComponents<RapierRigidBodyHandle>` cursor, so both copies process the same removal
    /// event. Without a guard, the PostUpdate copy walks all contexts via `find_context` after
    /// `init_rigid_bodies` has reinserted the body in the new context, finds the freshly-created
    /// `entity2body` entry, and deletes it — leaving the entity with a live
    /// `RapierRigidBodyHandle` component and no body in any rapier set.
    ///
    /// This test reproduces that pattern (link swap + handle removal) and asserts the body lives
    /// in the new context after a few app updates.
    #[test]
    pub fn body_survives_context_migration_via_handle_swap() {
        let mut app = App::new();
        app.add_plugins((
            TransformPlugin,
            TimePlugin,
            RapierPhysicsPlugin::<NoUserData>::default().in_fixed_schedule(),
        ));
        app.insert_resource(TimeUpdateStrategy::ManualDuration(
            std::time::Duration::from_secs_f32(1.0 / 60.0),
        ));
        app.insert_resource(Time::<Fixed>::from_hz(60.0));
        app.add_systems(Startup, setup_body);
        app.finish();

        // Run a few frames so the default context is created, `setup_body` runs, and the body is
        // registered in the default context's `entity2body`.
        for _ in 0..3 {
            app.update();
        }

        let body_entity = app
            .world_mut()
            .query_filtered::<Entity, With<MigrationTarget>>()
            .single(app.world())
            .expect("body entity exists");
        let default_ctx_entity = app
            .world_mut()
            .query_filtered::<Entity, With<DefaultRapierContext>>()
            .single(app.world())
            .expect("default context exists");

        assert!(
            app.world()
                .entity(default_ctx_entity)
                .get::<RapierRigidBodySet>()
                .unwrap()
                .entity2body()
                .contains_key(&body_entity),
            "pre-migration: body must be registered in the default context",
        );

        // Spawn a second context. `setup_rapier_configuration` will attach `RapierConfiguration`
        // on the next update.
        let new_ctx_entity = app
            .world_mut()
            .spawn(RapierContextSimulation::default())
            .id();
        app.update();

        // Migrate: swap the entity's `RapierContextEntityLink` and drop its handles. This is the
        // exact pattern that triggered the dual-`sync_removals` deletion bug.
        {
            let world = app.world_mut();
            let mut entity_mut = world.entity_mut(body_entity);
            entity_mut.insert(RapierContextEntityLink(new_ctx_entity));
            entity_mut.remove::<RapierRigidBodyHandle>();
            entity_mut.remove::<RapierColliderHandle>();
        }

        // Run enough frames so both `sync_removals` copies process the removal event and
        // `init_rigid_bodies` / `init_colliders` re-create the body in the new context.
        for _ in 0..3 {
            app.update();
        }

        let world = app.world();
        let new_set = world
            .entity(new_ctx_entity)
            .get::<RapierRigidBodySet>()
            .unwrap();
        let default_set = world
            .entity(default_ctx_entity)
            .get::<RapierRigidBodySet>()
            .unwrap();

        assert!(
            new_set.entity2body().contains_key(&body_entity),
            "body must live in the new context after migration; \
             a stale removal event must not delete the freshly-reinserted body",
        );
        assert!(
            !default_set.entity2body().contains_key(&body_entity),
            "body must no longer be registered in the original context",
        );

        let handle = world
            .entity(body_entity)
            .get::<RapierRigidBodyHandle>()
            .expect("entity must have a fresh `RapierRigidBodyHandle`");
        let mapped = new_set
            .entity2body()
            .get(&body_entity)
            .expect("new context's `entity2body` must contain the migrated entity");
        assert_eq!(
            handle.0, *mapped,
            "the entity's `RapierRigidBodyHandle` component must reference the new context's body",
        );

        return;

        #[derive(Component)]
        pub struct MigrationTarget;

        #[cfg(feature = "dim3")]
        fn cuboid(hx: Real, hy: Real, hz: Real) -> Collider {
            Collider::cuboid(hx, hy, hz)
        }
        #[cfg(feature = "dim2")]
        fn cuboid(hx: Real, hy: Real, _hz: Real) -> Collider {
            Collider::cuboid(hx, hy)
        }

        pub fn setup_body(mut commands: Commands) {
            commands.spawn((
                Transform::from_xyz(0.0, 0.0, 0.0),
                RigidBody::Dynamic,
                cuboid(0.5, 0.5, 0.5),
                MigrationTarget,
            ));
        }
    }
}
