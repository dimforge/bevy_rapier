use crate::math::{Real, Vect};
use bevy::prelude::{Entity, Event, EventWriter};
use rapier::dynamics::RigidBodySet;
use rapier::geometry::{
    ColliderHandle, ColliderSet, CollisionEvent as RapierCollisionEvent, CollisionEventFlags,
    ContactForceEvent as RapierContactForceEvent, ContactPair,
};
use rapier::pipeline::EventHandler;
use std::collections::HashMap;
use std::sync::RwLock;

#[cfg(doc)]
use crate::prelude::{ActiveEvents, ContactForceEventThreshold};

/// Events occurring when two colliders start or stop colliding
///
/// This will only get triggered if the entity has the
/// [`ActiveEvents::COLLISION_EVENTS`] flag enabled.
#[derive(Event, Copy, Clone, Debug, PartialEq, Eq)]
pub enum CollisionEvent {
    /// Event occurring when two colliders start colliding
    Started(Entity, Entity, CollisionEventFlags),
    /// Event occurring when two colliders stop colliding
    Stopped(Entity, Entity, CollisionEventFlags),
}

/// Event occurring when the sum of the magnitudes of the contact forces
/// between two colliders exceed a threshold ([`ContactForceEventThreshold`]).
///
/// This will only get triggered if the entity has the
/// [`ActiveEvents::CONTACT_FORCE_EVENTS`] flag enabled.
#[derive(Event, Copy, Clone, Debug, PartialEq)]
pub struct ContactForceEvent {
    /// The first collider involved in the contact.
    pub collider1: Entity,
    /// The second collider involved in the contact.
    pub collider2: Entity,
    /// The sum of all the forces between the two colliders.
    pub total_force: Vect,
    /// The sum of the magnitudes of each force between the two colliders.
    ///
    /// Note that this is **not** the same as the magnitude of `self.total_force`.
    /// Here we are summing the magnitude of all the forces, instead of taking
    /// the magnitude of their sum.
    pub total_force_magnitude: Real,
    /// The world-space (unit) direction of the force with strongest magnitude.
    pub max_force_direction: Vect,
    /// The magnitude of the largest force at a contact point of this contact pair.
    pub max_force_magnitude: Real,
}

// TODO: it may be more efficient to use crossbeam channel.
// However crossbeam channels cause a Segfault (I have not
// investigated how to reproduce this exactly to open an
// issue).
/// A set of queues collecting events emitted by the physics engine.
pub(crate) struct EventQueue<'a> {
    // Used to retrieve the entity of colliders that have been removed from the simulation
    // since the last physics step.
    pub deleted_colliders: &'a HashMap<ColliderHandle, Entity>,
    pub collision_events: RwLock<EventWriter<'a, CollisionEvent>>,
    pub contact_force_events: RwLock<EventWriter<'a, ContactForceEvent>>,
}

impl<'a> EventQueue<'a> {
    fn collider2entity(&self, colliders: &ColliderSet, handle: ColliderHandle) -> Entity {
        colliders
            .get(handle)
            .map(|co| Entity::from_bits(co.user_data as u64))
            .or_else(|| self.deleted_colliders.get(&handle).copied())
            .expect("Internal error: entity not found for collision event.")
    }
}

impl<'a> EventHandler for EventQueue<'a> {
    fn handle_collision_event(
        &self,
        _bodies: &RigidBodySet,
        colliders: &ColliderSet,
        event: RapierCollisionEvent,
        _: Option<&ContactPair>,
    ) {
        let event = match event {
            RapierCollisionEvent::Started(h1, h2, flags) => {
                let e1 = self.collider2entity(colliders, h1);
                let e2 = self.collider2entity(colliders, h2);
                CollisionEvent::Started(e1, e2, flags)
            }
            RapierCollisionEvent::Stopped(h1, h2, flags) => {
                let e1 = self.collider2entity(colliders, h1);
                let e2 = self.collider2entity(colliders, h2);
                CollisionEvent::Stopped(e1, e2, flags)
            }
        };

        if let Ok(mut events) = self.collision_events.write() {
            events.send(event);
        }
    }

    fn handle_contact_force_event(
        &self,
        dt: Real,
        _bodies: &RigidBodySet,
        colliders: &ColliderSet,
        contact_pair: &ContactPair,
        total_force_magnitude: Real,
    ) {
        let rapier_event =
            RapierContactForceEvent::from_contact_pair(dt, contact_pair, total_force_magnitude);
        let event = ContactForceEvent {
            collider1: self.collider2entity(colliders, rapier_event.collider1),
            collider2: self.collider2entity(colliders, rapier_event.collider2),
            total_force: rapier_event.total_force.into(),
            total_force_magnitude: rapier_event.total_force_magnitude,
            max_force_direction: rapier_event.max_force_direction.into(),
            max_force_magnitude: rapier_event.max_force_magnitude,
        };

        if let Ok(mut events) = self.contact_force_events.write() {
            events.send(event);
        }
    }
}

#[cfg(test)]
mod test {
    use bevy::{
        app::{App, Startup, Update},
        prelude::{Commands, Component, Entity, Query, With},
        time::{TimePlugin, TimeUpdateStrategy},
        transform::{bundles::TransformBundle, components::Transform, TransformPlugin},
        MinimalPlugins,
    };
    use systems::tests::HeadlessRenderPlugin;

    use crate::{plugin::*, prelude::*};

    #[cfg(feature = "dim3")]
    fn cuboid(hx: Real, hy: Real, hz: Real) -> Collider {
        Collider::cuboid(hx, hy, hz)
    }
    #[cfg(feature = "dim2")]
    fn cuboid(hx: Real, hy: Real, _hz: Real) -> Collider {
        Collider::cuboid(hx, hy)
    }

    #[test]
    pub fn events_received() {
        return main();

        use bevy::prelude::*;

        #[derive(Resource, Reflect)]
        pub struct EventsSaver<E: Event> {
            pub events: Vec<E>,
        }

        impl<E: Event> Default for EventsSaver<E> {
            fn default() -> Self {
                Self {
                    events: Default::default(),
                }
            }
        }

        pub fn save_events<E: Event + Clone>(
            mut events: EventReader<E>,
            mut saver: ResMut<EventsSaver<E>>,
        ) {
            for event in events.read() {
                saver.events.push(event.clone());
            }
        }

        fn run_test(app: &mut App) {
            app.add_systems(PostUpdate, save_events::<CollisionEvent>)
                .add_systems(PostUpdate, save_events::<ContactForceEvent>)
                .init_resource::<EventsSaver<CollisionEvent>>()
                .init_resource::<EventsSaver<ContactForceEvent>>();

            // Simulates 60 updates per seconds
            app.insert_resource(TimeUpdateStrategy::ManualDuration(
                std::time::Duration::from_secs_f32(1f32 / 60f32),
            ));
            // 2 seconds should be plenty of time for the cube to fall on the
            // lowest collider.
            for _ in 0..120 {
                app.update();
            }
            let saved_collisions = app
                .world()
                .get_resource::<EventsSaver<CollisionEvent>>()
                .unwrap();
            assert_eq!(saved_collisions.events.len(), 3);
            let saved_contact_forces = app
                .world()
                .get_resource::<EventsSaver<ContactForceEvent>>()
                .unwrap();
            assert_eq!(saved_contact_forces.events.len(), 1);
        }

        /// Adapted from events example
        fn main() {
            let mut app = App::new();
            app.add_plugins((
                HeadlessRenderPlugin,
                TransformPlugin,
                TimePlugin,
                RapierPhysicsPlugin::<NoUserData>::default(),
            ))
            .add_systems(Startup, setup_physics);
            run_test(&mut app);
        }

        pub fn setup_physics(mut commands: Commands) {
            /*
             * Ground
             */
            commands.spawn((
                TransformBundle::from(Transform::from_xyz(0.0, -1.2, 0.0)),
                cuboid(4.0, 1.0, 1.0),
            ));

            commands.spawn((
                TransformBundle::from(Transform::from_xyz(0.0, 5.0, 0.0)),
                cuboid(4.0, 1.5, 1.0),
                Sensor,
            ));

            commands.spawn((
                TransformBundle::from(Transform::from_xyz(0.0, 13.0, 0.0)),
                RigidBody::Dynamic,
                cuboid(0.5, 0.5, 0.5),
                ActiveEvents::COLLISION_EVENTS | ActiveEvents::CONTACT_FORCE_EVENTS,
                ContactForceEventThreshold(30.0),
            ));
        }
    }

    #[test]
    pub fn spam_remove_rapier_entity_interpolated() {
        let mut app = App::new();
        app.add_plugins((
            HeadlessRenderPlugin,
            MinimalPlugins,
            TransformPlugin,
            RapierPhysicsPlugin::<NoUserData>::default(),
        ))
        .insert_resource(RapierConfiguration {
            timestep_mode: TimestepMode::Interpolated {
                dt: 1.0 / 30.0,
                time_scale: 1.0,
                substeps: 2,
            },
            ..RapierConfiguration::new(1f32)
        })
        .add_systems(Startup, setup_physics)
        .add_systems(Update, remove_collider);
        // Simulates 60 updates per seconds
        app.insert_resource(TimeUpdateStrategy::ManualDuration(
            std::time::Duration::from_secs_f32(1f32 / 60f32),
        ));

        for i in 0..100 {
            dbg!(i);
            app.update();
        }
        return;

        #[derive(Component)]
        pub struct ToRemove;

        #[cfg(feature = "dim3")]
        fn cuboid(hx: Real, hy: Real, hz: Real) -> Collider {
            Collider::cuboid(hx, hy, hz)
        }
        #[cfg(feature = "dim2")]
        fn cuboid(hx: Real, hy: Real, _hz: Real) -> Collider {
            Collider::cuboid(hx, hy)
        }
        pub fn setup_physics(mut commands: Commands) {
            for _i in 0..100 {
                commands.spawn((
                    TransformBundle::from(Transform::from_xyz(0.0, 0.0, 0.0)),
                    RigidBody::Dynamic,
                    cuboid(0.5, 0.5, 0.5),
                    ActiveEvents::all(),
                    ToRemove,
                ));
            }
            /*
             * Ground
             */
            let ground_size = 5.1;
            let ground_height = 0.1;
            let starting_y = -0.5 - ground_height;

            commands.spawn((
                TransformBundle::from(Transform::from_xyz(0.0, starting_y, 0.0)),
                cuboid(ground_size, ground_height, ground_size),
            ));
        }

        fn remove_collider(mut commands: Commands, query: Query<Entity, With<ToRemove>>) {
            let Some(entity) = query.iter().next() else {
                return;
            };
            commands.entity(entity).despawn();
        }
    }
}
