use bevy::{ecs::system::SystemParam, prelude::*};
use rapier::prelude::{PhysicsHooks, SolverFlags};

pub use rapier::pipeline::{ContactModificationContext, PairFilterContext};

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
    fn filter_contact_pair(&self, _context: &PairFilterContext) -> Option<SolverFlags> {
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
    fn filter_intersection_pair(&self, _context: &PairFilterContext) -> bool {
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
    fn modify_solver_contacts(&self, _context: &mut ContactModificationContext) {}
}

impl<T> BevyPhysicsHooks for T
where
    T: 'static + PhysicsHooks + SystemParam + Send + Sync,
    for<'w, 's> T: SystemParam<Item<'w, 's> = T>,
{
    fn filter_contact_pair(&self, context: &PairFilterContext) -> Option<SolverFlags> {
        PhysicsHooks::filter_contact_pair(self, context)
    }

    fn filter_intersection_pair(&self, context: &PairFilterContext) -> bool {
        PhysicsHooks::filter_intersection_pair(self, context)
    }

    fn modify_solver_contacts(&self, context: &mut ContactModificationContext) {
        PhysicsHooks::modify_solver_contacts(self, context)
    }
}

/// Adapts a type implementing `BevyPhysicsHooks` so that it implements `PhysicsHooks`.
pub(crate) struct BevyPhysicsHooksAdapter<Hooks>
where
    Hooks: BevyPhysicsHooks,
{
    hooks: Hooks,
}

impl<Hooks> BevyPhysicsHooksAdapter<Hooks>
where
    Hooks: BevyPhysicsHooks,
{
    pub(crate) fn new(hooks: Hooks) -> Self {
        Self { hooks }
    }
}

impl<Hooks> PhysicsHooks for BevyPhysicsHooksAdapter<Hooks>
where
    Hooks: BevyPhysicsHooks,
{
    fn filter_contact_pair(&self, context: &PairFilterContext) -> Option<SolverFlags> {
        self.hooks.filter_contact_pair(context)
    }

    fn filter_intersection_pair(&self, context: &PairFilterContext) -> bool {
        self.hooks.filter_intersection_pair(context)
    }

    fn modify_solver_contacts(&self, context: &mut ContactModificationContext) {
        self.hooks.modify_solver_contacts(context)
    }
}
