use crate::math::{Real, Vect};
use bevy::prelude::{Entity, Event};
use rapier::dynamics::RigidBodySet;
use rapier::geometry::{
    ColliderHandle, ColliderSet, CollisionEvent as RapierCollisionEvent, CollisionEventFlags,
    ContactForceEvent as RapierContactForceEvent, ContactPair,
};
use rapier::pipeline::EventHandler;
use std::collections::HashMap;
use std::sync::RwLock;

/// Events occurring when two colliders start or stop colliding
#[derive(Event, Copy, Clone, Debug, PartialEq, Eq)]
pub enum CollisionEvent {
    /// Event occurring when two colliders start colliding
    Started(Entity, Entity, CollisionEventFlags),
    /// Event occurring when two colliders stop colliding
    Stopped(Entity, Entity, CollisionEventFlags),
}

/// Event occurring when the sum of the magnitudes of the contact forces
/// between two colliders exceed a threshold.
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
    pub collision_events: &'a mut RwLock<Vec<CollisionEvent>>,
    pub contact_force_events: &'a mut RwLock<Vec<ContactForceEvent>>,
}

impl<'a> EventQueue<'a> {
    fn collider2entity(&self, colliders: &ColliderSet, handle: ColliderHandle) -> Option<Entity> {
        colliders
            .get(handle)
            .map(|co| Entity::from_bits(co.user_data as u64))
            .or_else(|| self.deleted_colliders.get(&handle).copied())
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
                let Some(e1) = self.collider2entity(colliders, h1) else {
                    return;
                };
                let Some(e2) = self.collider2entity(colliders, h2) else {
                    return;
                };

                CollisionEvent::Started(e1, e2, flags)
            }
            RapierCollisionEvent::Stopped(h1, h2, flags) => {
                let Some(e1) = self.collider2entity(colliders, h1) else {
                    return;
                };
                let Some(e2) = self.collider2entity(colliders, h2) else {
                    return;
                };

                CollisionEvent::Stopped(e1, e2, flags)
            }
        };

        if let Ok(mut events) = self.collision_events.write() {
            events.push(event)
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

        let Some(collider1) = self.collider2entity(colliders, rapier_event.collider1) else {
            return;
        };
        let Some(collider2) = self.collider2entity(colliders, rapier_event.collider2) else {
            return;
        };

        let event = ContactForceEvent {
            collider1,
            collider2,
            total_force: rapier_event.total_force.into(),
            total_force_magnitude: rapier_event.total_force_magnitude,
            max_force_direction: rapier_event.max_force_direction.into(),
            max_force_magnitude: rapier_event.max_force_magnitude,
        };

        if let Ok(mut events) = self.contact_force_events.write() {
            events.push(event);
        }
    }
}
