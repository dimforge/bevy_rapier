use crate::rapier::geometry::{ContactEvent, ProximityEvent};
use crate::rapier::pipeline::EventHandler;
use bevy::ecs::Entity;
use concurrent_queue::ConcurrentQueue;
use rapier::dynamics::RigidBodyHandle;
use rapier::math::Vector;
use std::collections::HashMap;

#[derive(Copy, Clone)]
pub struct RapierPhysicsScale(pub f32);

pub struct EntityToBody(pub(crate) HashMap<Entity, RigidBodyHandle>);

impl EntityToBody {
    pub fn new() -> Self {
        EntityToBody(HashMap::new())
    }
}

/// A resource for specifying the gravity.
pub struct Gravity(pub Vector<f32>);

// TODO: it may be more efficient to use crossbeam channel.
// However crossbeam channels cause a Segfault (I have not
// investigated how to reproduce this exactly to open an
// issue).
pub struct EventQueue {
    pub contact_events: ConcurrentQueue<ContactEvent>,
    pub proximity_events: ConcurrentQueue<ProximityEvent>,
    pub auto_clear: bool,
}

impl EventQueue {
    pub fn new(auto_clear: bool) -> Self {
        Self {
            contact_events: ConcurrentQueue::unbounded(),
            proximity_events: ConcurrentQueue::unbounded(),
            auto_clear,
        }
    }

    /// Removes all events contained by this queue.
    pub fn clear(&self) {
        while let Ok(_) = self.contact_events.pop() {}
        while let Ok(_) = self.proximity_events.pop() {}
    }
}

impl EventHandler for EventQueue {
    fn handle_proximity_event(&self, event: ProximityEvent) {
        let _ = self.proximity_events.push(event);
    }

    fn handle_contact_event(&self, event: ContactEvent) {
        let _ = self.contact_events.push(event);
    }
}
