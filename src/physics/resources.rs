use crate::rapier::geometry::{ContactEvent, ProximityEvent};
use crate::rapier::pipeline::EventHandler;
use bevy::ecs::Entity;
use concurrent_queue::ConcurrentQueue;
use rapier::dynamics::RigidBodyHandle;
use rapier::math::Vector;
use std::collections::HashMap;

#[derive(Copy, Clone)]
/// A component describing a scale ration between the physics world and the bevy transforms.
///
/// This resource will affect the transform synchronization between Bevy and Rapier.
/// Each Rapier rigid-body position will have its coordinates multiplied by this scale factor.
pub struct RapierPhysicsScale(pub f32);

/// A map between bevy's entities and Rapier's handles.
///
/// This map will likely be removed in the future.
pub struct EntityToBody(pub(crate) HashMap<Entity, RigidBodyHandle>);

impl EntityToBody {
    /// Create a new empty map.
    pub fn new() -> Self {
        EntityToBody(HashMap::new())
    }
}

/// A resource for specifying the gravity of the physics simulation.
pub struct Gravity(pub Vector<f32>);

// TODO: it may be more efficient to use crossbeam channel.
// However crossbeam channels cause a Segfault (I have not
// investigated how to reproduce this exactly to open an
// issue).
/// A set of queues collecting events emitted by the physics engine.
pub struct EventQueue {
    /// The unbounded contact event queue.
    pub contact_events: ConcurrentQueue<ContactEvent>,
    /// The unbounded proximity event queue.
    pub proximity_events: ConcurrentQueue<ProximityEvent>,
    /// Are these queues automatically cleared before each simulation timestep?
    pub auto_clear: bool,
}

impl EventQueue {
    /// Creates a new empty event queue.
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
