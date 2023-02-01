use bevy::{
    ecs::system::{SystemParam, SystemParamFetch, SystemParamItem},
    prelude::*,
};
use rapier::{
    pipeline::{ContactModificationContext, PairFilterContext},
    prelude::{PhysicsHooks, SolverFlags},
};

/// Read-only access to the properties of a collision pair filter context.
pub struct PairFilterContextView<'a> {
    /// The raw context from Rapier.
    pub raw: &'a PairFilterContext<'a>,
}

impl<'a> PairFilterContextView<'a> {
    /// The entity of the first collider involved in the potential collision.
    pub fn collider1(&self) -> Entity {
        let co1 = &self.raw.colliders[self.raw.collider1];
        Entity::from_bits(co1.user_data as u64)
    }

    /// The entity of the second collider involved in the potential collision.
    pub fn collider2(&self) -> Entity {
        let co2 = &self.raw.colliders[self.raw.collider2];
        Entity::from_bits(co2.user_data as u64)
    }

    /// The entity of the first rigid-body (if `self.collider1()` is attached to a rigid-body)
    /// involved in the potential collision.
    pub fn rigid_body1(&self) -> Option<Entity> {
        self.raw.rigid_body1.map(|h| {
            let co1 = &self.raw.bodies[h];
            Entity::from_bits(co1.user_data as u64)
        })
    }

    /// The entity of the second rigid-body (if `self.collider1()` is attached to a rigid-body)
    /// involved in the potential collision.
    pub fn rigid_body2(&self) -> Option<Entity> {
        self.raw.rigid_body2.map(|h| {
            let co2 = &self.raw.bodies[h];
            Entity::from_bits(co2.user_data as u64)
        })
    }
}

/// Read-write access to the properties of a contact modification context.
pub struct ContactModificationContextView<'a, 'b> {
    /// The raw context from Rapier.
    pub raw: &'a mut ContactModificationContext<'b>,
}

impl<'a, 'b> ContactModificationContextView<'a, 'b> {
    /// The entity of the first collider involved in the potential collision.
    pub fn collider1(&self) -> Entity {
        let co1 = &self.raw.colliders[self.raw.collider1];
        Entity::from_bits(co1.user_data as u64)
    }

    /// The entity of the second collider involved in the potential collision.
    pub fn collider2(&self) -> Entity {
        let co2 = &self.raw.colliders[self.raw.collider2];
        Entity::from_bits(co2.user_data as u64)
    }

    /// The entity of the first rigid-body (if `self.collider1()` is attached to a rigid-body)
    /// involved in the potential collision.
    pub fn rigid_body1(&self) -> Option<Entity> {
        self.raw.rigid_body1.map(|h| {
            let co1 = &self.raw.bodies[h];
            Entity::from_bits(co1.user_data as u64)
        })
    }

    /// The entity of the second rigid-body (if `self.collider1()` is attached to a rigid-body)
    /// involved in the potential collision.
    pub fn rigid_body2(&self) -> Option<Entity> {
        self.raw.rigid_body2.map(|h| {
            let co2 = &self.raw.bodies[h];
            Entity::from_bits(co2.user_data as u64)
        })
    }
}

/// User-defined functions called by the physics engines during one timestep in order to customize its behavior.
pub trait BevyPhysicsHooks: SystemParam + Send + Sync {
    /// Applies the contact pair filter.
    ///
    /// Note that this method will only be called if at least one of the colliders
    /// involved in the contact contains the `ActiveHooks::FILTER_CONTACT_PAIRS` flags
    /// in its physics hooks flags.
    ///
    /// User-defined filter for potential contact pairs detected by the broad-phase.
    /// This can be used to apply custom logic in order to decide whether two colliders
    /// should have their contact computed by the narrow-phase, and if these contact
    /// should be solved by the constraints solver
    ///
    /// Note that using a contact pair filter will replace the default contact filtering
    /// which consists of preventing contact computation between two non-dynamic bodies.
    ///
    /// This filtering method is called after taking into account the colliders collision groups.
    ///
    /// If this returns `None`, then the narrow-phase will ignore this contact pair and
    /// not compute any contact manifolds for it.
    /// If this returns `Some`, then the narrow-phase will compute contact manifolds for
    /// this pair of colliders, and configure them with the returned solver flags. For
    /// example, if this returns `Some(SolverFlags::COMPUTE_IMPULSES)` then the contacts
    /// will be taken into account by the constraints solver. If this returns
    /// `Some(SolverFlags::empty())` then the constraints solver will ignore these
    /// contacts.
    fn filter_contact_pair(&self, _context: PairFilterContextView) -> Option<SolverFlags> {
        None
    }

    /// Applies the intersection pair filter.
    ///
    /// Note that this method will only be called if at least one of the colliders
    /// involved in the contact contains the `ActiveHooks::FILTER_INTERSECTION_PAIR` flags
    /// in its physics hooks flags.
    ///
    /// User-defined filter for potential intersection pairs detected by the broad-phase.
    ///
    /// This can be used to apply custom logic in order to decide whether two colliders
    /// should have their intersection computed by the narrow-phase.
    ///
    /// Note that using an intersection pair filter will replace the default intersection filtering
    /// which consists of preventing intersection computation between two non-dynamic bodies.
    ///
    /// This filtering method is called after taking into account the colliders collision groups.
    ///
    /// If this returns `false`, then the narrow-phase will ignore this pair and
    /// not compute any intersection information for it.
    /// If this return `true` then the narrow-phase will compute intersection
    /// information for this pair.
    fn filter_intersection_pair(&self, _context: PairFilterContextView) -> bool {
        false
    }

    /// Modifies the set of contacts seen by the constraints solver.
    ///
    /// Note that this method will only be called if at least one of the colliders
    /// involved in the contact contains the `ActiveHooks::MODIFY_SOLVER_CONTACTS` flags
    /// in its physics hooks flags.
    ///
    /// By default, the content of `solver_contacts` is computed from `manifold.points`.
    /// This method will be called on each contact manifold which have the flag `SolverFlags::modify_solver_contacts` set.
    /// This method can be used to modify the set of solver contacts seen by the constraints solver: contacts
    /// can be removed and modified.
    ///
    /// Note that if all the contacts have to be ignored by the constraint solver, you may simply
    /// do `context.solver_contacts.clear()`.
    ///
    /// Modifying the solver contacts allow you to achieve various effects, including:
    /// - Simulating conveyor belts by setting the `surface_velocity` of a solver contact.
    /// - Simulating shapes with multiply materials by modifying the friction and restitution
    ///   coefficient depending of the features in contacts.
    /// - Simulating one-way platforms depending on the contact normal.
    ///
    /// Each contact manifold is given a `u32` user-defined data that is persistent between
    /// timesteps (as long as the contact manifold exists). This user-defined data is initialized
    /// as 0 and can be modified in `context.user_data`.
    ///
    /// The world-space contact normal can be modified in `context.normal`.
    fn modify_solver_contacts(&self, _context: ContactModificationContextView) {}
}

impl<T> BevyPhysicsHooks for T
where
    T: 'static + PhysicsHooks + SystemParam + Send + Sync,
    for<'w, 's> T::Fetch: SystemParamFetch<'w, 's, Item = T>,
{
    fn filter_contact_pair(&self, context: PairFilterContextView) -> Option<SolverFlags> {
        PhysicsHooks::filter_contact_pair(self, context.raw)
    }

    fn filter_intersection_pair(&self, context: PairFilterContextView) -> bool {
        PhysicsHooks::filter_intersection_pair(self, context.raw)
    }

    fn modify_solver_contacts(&self, context: ContactModificationContextView) {
        PhysicsHooks::modify_solver_contacts(self, context.raw)
    }
}

/// Adapts a type implementing `BevyPhysicsHooks` so that it implements `PhysicsHooks`.
pub(crate) struct BevyPhysicsHooksAdapter<'w, 's, Hooks>
where
    Hooks: 'static + BevyPhysicsHooks,
    for<'w1, 's1> SystemParamItem<'w1, 's1, Hooks>: BevyPhysicsHooks,
{
    hooks: SystemParamItem<'w, 's, Hooks>,
}

impl<'w, 's, Hooks> BevyPhysicsHooksAdapter<'w, 's, Hooks>
where
    Hooks: 'static + BevyPhysicsHooks,
    for<'w1, 's1> SystemParamItem<'w1, 's1, Hooks>: BevyPhysicsHooks,
{
    pub(crate) fn new(hooks: SystemParamItem<'w, 's, Hooks>) -> Self {
        Self { hooks }
    }
}

impl<'w, 's, Hooks> PhysicsHooks for BevyPhysicsHooksAdapter<'w, 's, Hooks>
where
    Hooks: 'static + BevyPhysicsHooks,
    for<'w1, 's1> SystemParamItem<'w1, 's1, Hooks>: BevyPhysicsHooks,
{
    fn filter_contact_pair(&self, context: &PairFilterContext) -> Option<SolverFlags> {
        let context_view = PairFilterContextView { raw: context };
        self.hooks.filter_contact_pair(context_view)
    }

    fn filter_intersection_pair(&self, context: &PairFilterContext) -> bool {
        let context_view = PairFilterContextView { raw: context };
        self.hooks.filter_intersection_pair(context_view)
    }

    fn modify_solver_contacts(&self, context: &mut ContactModificationContext) {
        let context_view = ContactModificationContextView { raw: context };
        self.hooks.modify_solver_contacts(context_view)
    }
}
