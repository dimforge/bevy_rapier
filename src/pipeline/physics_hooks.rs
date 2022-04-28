use bevy::ecs::query::WorldQuery;
use bevy::prelude::*;
use rapier::geometry::SolverFlags;
use rapier::pipeline::{ContactModificationContext, PairFilterContext, PhysicsHooks};

pub struct PairFilterContextView<'a> {
    pub raw: &'a PairFilterContext<'a>,
}

impl<'a> PairFilterContextView<'a> {
    pub fn collider1(&self) -> Entity {
        let co1 = &self.raw.colliders[self.raw.collider1];
        Entity::from_bits(co1.user_data as u64)
    }

    pub fn collider2(&self) -> Entity {
        let co2 = &self.raw.colliders[self.raw.collider2];
        Entity::from_bits(co2.user_data as u64)
    }

    pub fn rigid_body1(&self) -> Option<Entity> {
        self.raw.rigid_body1.map(|h| {
            let co1 = &self.raw.bodies[h];
            Entity::from_bits(co1.user_data as u64)
        })
    }

    pub fn rigid_body2(&self) -> Option<Entity> {
        self.raw.rigid_body2.map(|h| {
            let co2 = &self.raw.bodies[h];
            Entity::from_bits(co2.user_data as u64)
        })
    }
}

pub struct ContactModificationContextView<'a, 'b> {
    pub raw: &'a mut ContactModificationContext<'b>,
}

impl<'a, 'b> ContactModificationContextView<'a, 'b> {
    pub fn collider1(&self) -> Entity {
        let co1 = &self.raw.colliders[self.raw.collider1];
        Entity::from_bits(co1.user_data as u64)
    }

    pub fn collider2(&self) -> Entity {
        let co2 = &self.raw.colliders[self.raw.collider2];
        Entity::from_bits(co2.user_data as u64)
    }

    pub fn rigid_body1(&self) -> Option<Entity> {
        self.raw.rigid_body1.map(|h| {
            let co1 = &self.raw.bodies[h];
            Entity::from_bits(co1.user_data as u64)
        })
    }

    pub fn rigid_body2(&self) -> Option<Entity> {
        self.raw.rigid_body2.map(|h| {
            let co2 = &self.raw.bodies[h];
            Entity::from_bits(co2.user_data as u64)
        })
    }
}

pub trait PhysicsHooksWithQuery<UserData: WorldQuery>: Send + Sync {
    fn filter_contact_pair(
        &self,
        _context: PairFilterContextView,
        _user_data: &Query<UserData>,
    ) -> Option<SolverFlags> {
        None
    }

    fn filter_intersection_pair(
        &self,
        _context: PairFilterContextView,
        _user_data: &Query<UserData>,
    ) -> bool {
        false
    }

    fn modify_solver_contacts(
        &self,
        _context: ContactModificationContextView,
        _user_data: &Query<UserData>,
    ) {
    }
}

impl<T, UserData> PhysicsHooksWithQuery<UserData> for T
where
    T: PhysicsHooks + Send + Sync,
    UserData: WorldQuery,
{
    fn filter_contact_pair(
        &self,
        context: PairFilterContextView,
        _: &Query<UserData>,
    ) -> Option<SolverFlags> {
        PhysicsHooks::filter_contact_pair(self, context.raw)
    }

    fn filter_intersection_pair(
        &self,
        context: PairFilterContextView,
        _: &Query<UserData>,
    ) -> bool {
        PhysicsHooks::filter_intersection_pair(self, context.raw)
    }

    fn modify_solver_contacts(&self, context: ContactModificationContextView, _: &Query<UserData>) {
        PhysicsHooks::modify_solver_contacts(self, context.raw)
    }
}

pub struct PhysicsHooksWithQueryResource<UserData: WorldQuery>(
    pub Box<dyn PhysicsHooksWithQuery<UserData>>,
);

pub(crate) struct PhysicsHooksWithQueryInstance<'world, 'state, 'b, UserData: WorldQuery> {
    // pub commands: Commands<'world, 'state>,
    pub user_data: Query<'world, 'state, UserData>,
    pub hooks: &'b dyn PhysicsHooksWithQuery<UserData>,
}

impl<UserData: WorldQuery> PhysicsHooks for PhysicsHooksWithQueryInstance<'_, '_, '_, UserData> {
    fn filter_contact_pair(&self, context: &PairFilterContext) -> Option<SolverFlags> {
        let context_view = PairFilterContextView { raw: context };
        self.hooks
            .filter_contact_pair(context_view, &self.user_data)
    }

    fn filter_intersection_pair(&self, context: &PairFilterContext) -> bool {
        let context_view = PairFilterContextView { raw: context };
        self.hooks
            .filter_intersection_pair(context_view, &self.user_data)
    }

    fn modify_solver_contacts(&self, context: &mut ContactModificationContext) {
        let context_view = ContactModificationContextView { raw: context };
        self.hooks
            .modify_solver_contacts(context_view, &self.user_data)
    }
}
