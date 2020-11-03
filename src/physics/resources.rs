use crate::rapier::geometry::{
    ContactEvent, ContactPairFilter, ProximityEvent, ProximityPairFilter,
};
use crate::rapier::pipeline::EventHandler;
use concurrent_queue::ConcurrentQueue;
use rapier::math::Vector;

/// A resource for specifying configuration information for the physics simulation
pub struct RapierConfiguration {
    /// Specifying the gravity of the physics simulation.
    pub gravity: Vector<f32>,
    /// Specifies a scale ratio between the physics world and the bevy transforms.
    /// This will affect the transform synchronization between Bevy and Rapier.
    /// Each Rapier rigid-body position will have its coordinates multiplied by this scale factor.
    pub scale: f32,
    /// Specifies if the physics simulation is active and update the physics world.
    pub physics_pipeline_active: bool,
    /// Specifies if the query pipeline is active and update the query pipeline.
    pub query_pipeline_active: bool,
}

impl Default for RapierConfiguration {
    fn default() -> Self {
        Self {
            gravity: Vector::y() * -9.81,
            scale: 1.0,
            physics_pipeline_active: true,
            query_pipeline_active: true,
        }
    }
}

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

/// Difference between simulation and rendering time
#[derive(Default)]
pub struct SimulationToRenderTime {
    /// Difference between simulation and rendering time
    pub diff: f32,
}

/// Custom filters for proximity and contact pairs.
pub struct InteractionPairFilters {
    /// Custom proximity pair filter.
    pub proximity_filter: Option<Box<dyn ProximityPairFilter>>,
    /// Custom contact pair filter.
    pub contact_filter: Option<Box<dyn ContactPairFilter>>,
}

impl InteractionPairFilters {
    /// A new interaction pair filter with no custom proximity and contact pair filters.
    pub fn new() -> Self {
        Self {
            proximity_filter: None,
            contact_filter: None,
        }
    }

    /// Sets the custom contact pair filter.
    pub fn contact_filter(mut self, filter: impl ContactPairFilter + 'static) -> Self {
        self.contact_filter = Some(Box::new(filter) as Box<dyn ContactPairFilter>);
        self
    }

    /// Sets the custom proximity pair filter.
    pub fn proximity_filter(mut self, filter: impl ProximityPairFilter + 'static) -> Self {
        self.proximity_filter = Some(Box::new(filter) as Box<dyn ProximityPairFilter>);
        self
    }
}
