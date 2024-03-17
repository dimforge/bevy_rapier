use crate::math::{Real, Vect};
use bevy::prelude::{Entity, Event, EventWriter};
use rapier::dynamics::RigidBodySet;
use rapier::geometry::{ColliderHandle, ColliderSet, CollisionEventFlags, ContactPair};
use rapier::pipeline::EventHandler;
use std::collections::{HashMap, HashSet};
use std::sync::RwLock;

pub use rapier::geometry::{CollisionEvent, ContactForceEvent};

// TODO: it may be more efficient to use crossbeam channel.
// However crossbeam channels cause a Segfault (I have not
// investigated how to reproduce this exactly to open an
// issue).
/// A set of queues collecting events emitted by the physics engine.
pub(crate) struct EventQueue<'a> {
    // Used ot retrieve the entity of colliders that have been removed from the simulation
    // since the last physics step.
    pub deleted_colliders: &'a HashSet<Entity>,
    pub collision_events: RwLock<EventWriter<'a, CollisionEvent>>,
    pub contact_force_events: RwLock<EventWriter<'a, ContactForceEvent>>,
}

impl<'a> EventHandler for EventQueue<'a> {
    fn handle_collision_event(
        &self,
        _bodies: &RigidBodySet,
        colliders: &ColliderSet,
        event: CollisionEvent,
        _: Option<&ContactPair>,
    ) {
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
        let event = ContactForceEvent::from_contact_pair(dt, contact_pair, total_force_magnitude);
        if let Ok(mut events) = self.contact_force_events.write() {
            events.send(event);
        }
    }
}
