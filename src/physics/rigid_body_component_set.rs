use super::{IntoEntity, IntoHandle};
use crate::rapier::dynamics::{
    RigidBodyActivation, RigidBodyCcd, RigidBodyChanges, RigidBodyColliders, RigidBodyDamping,
    RigidBodyDominance, RigidBodyForces, RigidBodyHandle, RigidBodyIds, RigidBodyMassProps,
    RigidBodyPosition, RigidBodyType, RigidBodyVelocity,
};
use bevy::prelude::*;
use rapier::data::{ComponentSet, ComponentSetMut, ComponentSetOption, Index};

impl IntoHandle<RigidBodyHandle> for Entity {
    #[inline]
    fn handle(self) -> RigidBodyHandle {
        RigidBodyHandle::from_raw_parts(self.id(), self.generation())
    }
}

impl IntoEntity for RigidBodyHandle {
    #[inline]
    fn entity(self) -> Entity {
        self.0.entity()
    }
}

pub type RigidBodyComponentsQueryPayload<'a> = (
    Entity,
    &'a mut RigidBodyPosition,
    &'a mut RigidBodyVelocity,
    &'a mut RigidBodyMassProps,
    &'a mut RigidBodyIds,
    &'a mut RigidBodyForces,
    &'a mut RigidBodyCcd,
    &'a mut RigidBodyColliders,
    &'a mut RigidBodyDamping,
    &'a mut RigidBodyDominance,
    &'a mut RigidBodyType,
    &'a mut RigidBodyChanges,
    &'a mut RigidBodyActivation,
);

pub type RigidBodyChangesQueryPayload<'a> = (
    Entity,
    &'a mut RigidBodyActivation,
    &'a mut RigidBodyChanges,
    Or<(Changed<RigidBodyPosition>, Added<RigidBodyPosition>)>,
    Or<(Changed<RigidBodyType>, Added<RigidBodyType>)>,
    Or<(Changed<RigidBodyColliders>, Added<RigidBodyColliders>)>,
);

pub type RigidBodyChangesQueryFilter = (
    Or<(
        Changed<RigidBodyPosition>,
        Added<RigidBodyPosition>,
        Changed<RigidBodyVelocity>,
        Added<RigidBodyVelocity>,
        Changed<RigidBodyForces>,
        Added<RigidBodyForces>,
        Changed<RigidBodyActivation>,
        Added<RigidBodyActivation>,
        Changed<RigidBodyType>,
        Added<RigidBodyType>,
        Changed<RigidBodyColliders>,
        Added<RigidBodyColliders>,
    )>,
);

pub type RigidBodyComponentsQuerySet<'world, 'state, 'a> = QuerySet<
    'world,
    'state,
    (
        // Components query
        QueryState<RigidBodyComponentsQueryPayload<'a>>,
        // Changes query
        QueryState<RigidBodyChangesQueryPayload<'a>, RigidBodyChangesQueryFilter>,
    ),
>;

pub struct RigidBodyComponentsSet<'world, 'state, 'a>(
    pub Query<'world, 'state, RigidBodyComponentsQueryPayload<'a>>,
);

impl_component_set_mut!(RigidBodyComponentsSet, RigidBodyPosition, |data| &*data.1);
impl_component_set_mut!(RigidBodyComponentsSet, RigidBodyVelocity, |data| &*data.2);
impl_component_set_mut!(RigidBodyComponentsSet, RigidBodyMassProps, |data| &*data.3);
impl_component_set_mut!(RigidBodyComponentsSet, RigidBodyIds, |data| &*data.4);
impl_component_set_mut!(RigidBodyComponentsSet, RigidBodyForces, |data| &*data.5);
impl_component_set_mut!(RigidBodyComponentsSet, RigidBodyCcd, |data| &*data.6);
impl_component_set_mut!(RigidBodyComponentsSet, RigidBodyColliders, |data| &*data.7);
impl_component_set_mut!(RigidBodyComponentsSet, RigidBodyDamping, |data| &*data.8);
impl_component_set_mut!(RigidBodyComponentsSet, RigidBodyDominance, |data| &*data.9);
impl_component_set_mut!(RigidBodyComponentsSet, RigidBodyType, |data| &*data.10);
impl_component_set_mut!(RigidBodyComponentsSet, RigidBodyChanges, |data| &*data.11);
impl_component_set_mut!(RigidBodyComponentsSet, RigidBodyActivation, |data| &*data
    .12);

#[derive(Bundle)]
pub struct RigidBodyBundle {
    pub body_type: RigidBodyType,
    pub position: RigidBodyPosition,
    pub velocity: RigidBodyVelocity,
    pub mass_properties: RigidBodyMassProps,
    pub forces: RigidBodyForces,
    pub activation: RigidBodyActivation,
    pub damping: RigidBodyDamping,
    pub dominance: RigidBodyDominance,
    pub ccd: RigidBodyCcd,
    pub changes: RigidBodyChanges,
    pub ids: RigidBodyIds,
    pub colliders: RigidBodyColliders,
}

impl Default for RigidBodyBundle {
    fn default() -> Self {
        Self {
            body_type: RigidBodyType::Dynamic,
            position: RigidBodyPosition::default(),
            velocity: RigidBodyVelocity::default(),
            mass_properties: RigidBodyMassProps::default(),
            forces: RigidBodyForces::default(),
            activation: RigidBodyActivation::default(),
            damping: RigidBodyDamping::default(),
            dominance: RigidBodyDominance::default(),
            ccd: RigidBodyCcd::default(),
            changes: RigidBodyChanges::default(),
            ids: RigidBodyIds::default(),
            colliders: RigidBodyColliders::default(),
        }
    }
}
