use crate::physics::{
    ColliderBundle, ColliderComponentsQuery, ColliderComponentsSet, IntoEntity, IntoHandle,
    JointHandleComponent, RigidBodyComponentsQuery, RigidBodyComponentsSet,
};
use crate::rapier::data::{ComponentSet, ComponentSetMut};
use crate::rapier::dynamics::{
    IslandManager, JointSet, RigidBodyActivation, RigidBodyChanges, RigidBodyColliders,
    RigidBodyIds, RigidBodyType,
};
use crate::rapier::geometry::{ColliderChanges, SolverFlags};
use crate::rapier::pipeline::{ContactModificationContext, PairFilterContext, PhysicsHooksFlags};
use crate::rapier::{
    dynamics::{JointHandle, RigidBodyHandle},
    geometry::{ColliderHandle, ContactEvent, IntersectionEvent},
    pipeline::{EventHandler, PhysicsHooks},
};
use bevy::ecs::query::WorldQuery;
use bevy::prelude::*;
use concurrent_queue::ConcurrentQueue;
use rapier::math::Vector;
use std::collections::HashMap;

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
    /// Specifies if the number of physics steps run at each frame should depend
    /// of the real-world time elapsed since the last step.
    pub time_dependent_number_of_timesteps: bool,
}

impl Default for RapierConfiguration {
    fn default() -> Self {
        Self {
            gravity: Vector::y() * -9.81,
            scale: 1.0,
            physics_pipeline_active: true,
            query_pipeline_active: true,
            time_dependent_number_of_timesteps: false,
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
    /// The unbounded intersection event queue.
    pub intersection_events: ConcurrentQueue<IntersectionEvent>,
    /// Are these queues automatically cleared before each simulation timestep?
    pub auto_clear: bool,
}

impl EventQueue {
    /// Creates a new empty event queue.
    pub fn new(auto_clear: bool) -> Self {
        Self {
            contact_events: ConcurrentQueue::unbounded(),
            intersection_events: ConcurrentQueue::unbounded(),
            auto_clear,
        }
    }

    /// Removes all events contained by this queue.
    pub fn clear(&self) {
        while let Ok(_) = self.contact_events.pop() {}
        while let Ok(_) = self.intersection_events.pop() {}
    }
}

impl EventHandler for EventQueue {
    fn handle_intersection_event(&self, event: IntersectionEvent) {
        let _ = self.intersection_events.push(event);
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

/// HashMaps of Bevy Entity to Rapier handles
#[derive(Default)]
pub struct JointsEntityMap(pub(crate) HashMap<Entity, JointHandle>);

pub struct ModificationTracker {
    pub(crate) modified_bodies: Vec<RigidBodyHandle>,
    pub(crate) modified_colliders: Vec<ColliderHandle>,
    pub(crate) removed_bodies: Vec<RigidBodyHandle>,
    pub(crate) removed_colliders: Vec<ColliderHandle>,
    // NOTE: right now, this actually contains an Entity instead of the JointHandle.
    //       but we will switch to JointHandle soon.
    pub(crate) removed_joints: Vec<JointHandle>,
    // We need to maintain these two because we have to access them
    // when an entity containing a collider/rigid-body has been despawn.
    pub(crate) body_colliders: HashMap<RigidBodyHandle, Vec<ColliderHandle>>,
    pub(crate) colliders_parent: HashMap<ColliderHandle, RigidBodyHandle>,
}

impl Default for ModificationTracker {
    fn default() -> Self {
        Self {
            modified_bodies: vec![],
            modified_colliders: vec![],
            removed_bodies: vec![],
            removed_colliders: vec![],
            removed_joints: vec![],
            body_colliders: HashMap::new(),
            colliders_parent: HashMap::new(),
        }
    }
}

impl ModificationTracker {
    pub fn clear_modified_and_removed(&mut self) {
        self.modified_colliders.clear();
        self.modified_bodies.clear();
        self.removed_bodies.clear();
        self.removed_colliders.clear();
        self.removed_joints.clear();
    }

    pub fn detect_modifications(
        &mut self,
        bodies_query: &mut RigidBodyComponentsQuery,
        colliders_query: &mut ColliderComponentsQuery,
    ) {
        // Detect modifications.
        for (
            entity,
            mut rb_changes,
            mut rb_activation,
            rb_pos,
            _rb_vels,
            _rb_forces,
            rb_type,
            rb_colliders,
        ) in bodies_query.q2_mut().iter_mut()
        {
            if !rb_changes.contains(RigidBodyChanges::MODIFIED) {
                self.modified_bodies.push(entity.handle());
            }

            *rb_changes |= RigidBodyChanges::MODIFIED;

            if rb_pos {
                *rb_changes |= RigidBodyChanges::POSITION;
            }
            if rb_type {
                *rb_changes |= RigidBodyChanges::TYPE;
            }
            if rb_colliders {
                *rb_changes |= RigidBodyChanges::COLLIDERS;
            }

            // Wake-up the rigid-body.
            *rb_changes |= RigidBodyChanges::SLEEP;
            rb_activation.wake_up(true);
        }

        for mut rb_changes in bodies_query.q3_mut().iter_mut() {
            *rb_changes |= RigidBodyChanges::SLEEP;
        }

        for (entity, mut co_changes, co_pos, co_groups, co_shape, co_type, co_parent) in
            colliders_query.q2_mut().iter_mut()
        {
            if !co_changes.contains(ColliderChanges::MODIFIED) {
                self.modified_colliders.push(entity.handle());
            }

            *co_changes |= ColliderChanges::MODIFIED;

            if co_pos {
                *co_changes |= ColliderChanges::POSITION;
            }
            if co_groups {
                *co_changes |= ColliderChanges::GROUPS;
            }
            if co_shape {
                *co_changes |= ColliderChanges::SHAPE;
            }
            if co_type {
                *co_changes |= ColliderChanges::TYPE;
            }
            if co_parent == Some(true) {
                *co_changes |= ColliderChanges::PARENT;
            }
        }
    }

    pub fn detect_removals(
        &mut self,
        removed_bodies: RemovedComponents<RigidBodyChanges>,
        removed_colliders: RemovedComponents<ColliderChanges>,
        removed_joints: RemovedComponents<JointHandleComponent>,
    ) {
        self.removed_bodies.extend(
            removed_bodies
                .iter()
                .map(|e| IntoHandle::<RigidBodyHandle>::handle(e)),
        );
        self.removed_colliders.extend(
            removed_colliders
                .iter()
                .map(|e| IntoHandle::<ColliderHandle>::handle(e)),
        );
        self.removed_joints.extend(
            removed_joints
                .iter()
                .map(|e| IntoHandle::<JointHandle>::handle(e)),
        );
    }

    pub fn propagate_removals<Bodies>(
        &mut self,
        commands: &mut Commands,
        islands: &mut IslandManager,
        bodies: &mut Bodies,
        joints: &mut JointSet,
        joints_map: &mut JointsEntityMap,
    ) where
        Bodies: ComponentSetMut<RigidBodyChanges>
            + ComponentSetMut<RigidBodyColliders>
            + ComponentSetMut<RigidBodyActivation> // Needed for joint removal.
            + ComponentSetMut<RigidBodyIds> // Needed for joint removal.
            + ComponentSet<RigidBodyType>, // Needed for joint removal.
    {
        for removed_body in self.removed_bodies.iter() {
            if let Some(colliders) = self.body_colliders.remove(removed_body) {
                for collider in colliders {
                    commands
                        .entity(collider.entity())
                        .remove_bundle::<ColliderBundle>();
                    self.removed_colliders.push(collider);
                }
            }
        }

        for removed_collider in self.removed_colliders.iter() {
            if let Some(parent) = self.colliders_parent.remove(removed_collider) {
                let rb_changes: Option<RigidBodyChanges> = bodies.get(parent.0).copied();

                if let Some(mut rb_changes) = rb_changes {
                    // Keep track of the fact the collider will be modified.
                    if !rb_changes.contains(RigidBodyChanges::MODIFIED) {
                        self.modified_bodies.push(parent);
                    }

                    // Detach the collider from the rigid-body.
                    bodies.map_mut_internal(parent.0, |rb_colliders: &mut RigidBodyColliders| {
                        rb_colliders.detach_collider(&mut rb_changes, *removed_collider);
                    });

                    // Set the new rigid-body changes flags.
                    bodies.set_internal(parent.0, rb_changes);

                    // Update the body's colliders map `self.body_colliders`.
                    let body_colliders = self.body_colliders.get_mut(&parent).unwrap();
                    if let Some(i) = body_colliders.iter().position(|c| *c == *removed_collider) {
                        body_colliders.swap_remove(i);
                    }
                }
            }
        }

        for removed_joints in self.removed_joints.iter() {
            let joint_handle = joints_map.0.remove(&removed_joints.entity());
            if let Some(joint_handle) = joint_handle {
                joints.remove(joint_handle, islands, bodies, true);
            }
        }
    }
}

pub trait PhysicsHooksWithQuery<UserData: WorldQuery>: Send + Sync {
    fn active_hooks(&self, user_data: &Query<UserData>) -> PhysicsHooksFlags;

    fn filter_contact_pair(
        &self,
        _context: &PairFilterContext<RigidBodyComponentsSet, ColliderComponentsSet>,
        _user_data: &Query<UserData>,
    ) -> Option<SolverFlags> {
        None
    }

    fn filter_intersection_pair(
        &self,
        _context: &PairFilterContext<RigidBodyComponentsSet, ColliderComponentsSet>,
        _user_data: &Query<UserData>,
    ) -> bool {
        false
    }

    fn modify_solver_contacts(
        &self,
        _context: &mut ContactModificationContext<RigidBodyComponentsSet, ColliderComponentsSet>,
        _user_data: &Query<UserData>,
    ) {
    }
}

impl<T, UserData> PhysicsHooksWithQuery<UserData> for T
where
    T: for<'a, 'b, 'c, 'd, 'e, 'f> PhysicsHooks<
        RigidBodyComponentsSet<'a, 'b, 'c>,
        ColliderComponentsSet<'d, 'e, 'f>,
    >,
    UserData: WorldQuery,
{
    fn active_hooks(&self, _: &Query<UserData>) -> PhysicsHooksFlags {
        PhysicsHooks::active_hooks(self)
    }

    fn filter_intersection_pair(
        &self,
        context: &PairFilterContext<RigidBodyComponentsSet, ColliderComponentsSet>,
        _: &Query<UserData>,
    ) -> bool {
        PhysicsHooks::filter_intersection_pair(self, context)
    }

    fn modify_solver_contacts(
        &self,
        context: &mut ContactModificationContext<RigidBodyComponentsSet, ColliderComponentsSet>,
        _: &Query<UserData>,
    ) {
        PhysicsHooks::modify_solver_contacts(self, context)
    }
}

pub struct PhysicsHooksWithQueryObject<UserData: WorldQuery>(
    pub Box<dyn PhysicsHooksWithQuery<UserData>>,
);

pub(crate) struct PhysicsHooksWithQueryInstance<'a, 'b, UserData: WorldQuery> {
    pub user_data: Query<'a, UserData>,
    pub hooks: &'b dyn PhysicsHooksWithQuery<UserData>,
}

impl<'aa, 'bb, 'a, 'b, 'c, 'd, 'e, 'f, UserData: WorldQuery>
    PhysicsHooks<RigidBodyComponentsSet<'a, 'b, 'c>, ColliderComponentsSet<'d, 'e, 'f>>
    for PhysicsHooksWithQueryInstance<'aa, 'bb, UserData>
{
    fn active_hooks(&self) -> PhysicsHooksFlags {
        self.hooks.active_hooks(&self.user_data)
    }

    fn filter_contact_pair(
        &self,
        context: &PairFilterContext<RigidBodyComponentsSet, ColliderComponentsSet>,
    ) -> Option<SolverFlags> {
        self.hooks.filter_contact_pair(context, &self.user_data)
    }

    fn filter_intersection_pair(
        &self,
        context: &PairFilterContext<RigidBodyComponentsSet, ColliderComponentsSet>,
    ) -> bool {
        self.hooks
            .filter_intersection_pair(context, &self.user_data)
    }

    fn modify_solver_contacts(
        &self,
        context: &mut ContactModificationContext<RigidBodyComponentsSet, ColliderComponentsSet>,
    ) {
        self.hooks.modify_solver_contacts(context, &self.user_data)
    }
}
