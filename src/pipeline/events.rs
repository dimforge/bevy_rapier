use bevy::prelude::{Entity, EventWriter};
use rapier::dynamics::RigidBodySet;
use rapier::geometry::{
    ColliderHandle, ColliderSet, CollisionEvent as RapierCollisionEvent, CollisionEventFlags,
    ContactPair,
};
use rapier::pipeline::EventHandler;
use std::collections::HashMap;
use std::sync::RwLock;

/// Events occurring when two colliders start or stop colliding
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum CollisionEvent {
    /// Event occurring when two colliders start colliding
    Started(Entity, Entity, CollisionEventFlags),
    /// Event occurring when two colliders stop colliding
    Stopped(Entity, Entity, CollisionEventFlags),
}

// TODO: it may be more efficient to use crossbeam channel.
// However crossbeam channels cause a Segfault (I have not
// investigated how to reproduce this exactly to open an
// issue).
/// A set of queues collecting events emitted by the physics engine.
pub(crate) struct EventQueue<'a> {
    // Used ot retrieve the entity of colliders that have been removed from the simulation
    // since the last physics step.
    pub deleted_colliders: &'a HashMap<ColliderHandle, Entity>,
    pub events: RwLock<EventWriter<'a, 'a, CollisionEvent>>,
}

impl<'a> EventHandler for EventQueue<'a> {
    fn handle_collision_event(
        &self,
        _bodies: &RigidBodySet,
        colliders: &ColliderSet,
        event: RapierCollisionEvent,
        _: Option<&ContactPair>,
    ) {
        let collider2entity = |handle| {
            colliders
                .get(handle)
                .map(|co| Entity::from_bits(co.user_data as u64))
                .or_else(|| self.deleted_colliders.get(&handle).copied())
                .expect("Internal error: entity not found for collision event.")
        };

        if let Ok(mut events) = self.events.write() {
            let event = match event {
                RapierCollisionEvent::Started(h1, h2, flags) => {
                    let e1 = collider2entity(h1);
                    let e2 = collider2entity(h2);
                    CollisionEvent::Started(e1, e2, flags)
                }
                RapierCollisionEvent::Stopped(h1, h2, flags) => {
                    let e1 = collider2entity(h1);
                    let e2 = collider2entity(h2);
                    CollisionEvent::Stopped(e1, e2, flags)
                }
            };
            events.send(event)
        }
    }
}
