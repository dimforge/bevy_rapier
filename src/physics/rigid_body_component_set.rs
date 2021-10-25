use super::{IntoEntity, IntoHandle};
use crate::physics::wrapper::{
    RigidBodyActivation, RigidBodyCcd, RigidBodyChanges, RigidBodyDamping,
    RigidBodyDominance, RigidBodyForces, RigidBodyHandle, RigidBodyIds, RigidBodyMassProps,
    RigidBodyPosition, RigidBodyType, RigidBodyVelocity,RigidBodyColliders
};
use bevy::prelude::*;
use rapier::data::{ComponentSet, ComponentSetMut, ComponentSetOption, Index};
use rapier::{dynamics};
impl IntoHandle<dynamics::RigidBodyHandle> for Entity {
    #[inline]
    fn handle(self) -> dynamics::RigidBodyHandle {
      dynamics::RigidBodyHandle::from_raw_parts(self.id(), self.generation())
    }
}

impl IntoEntity for dynamics::RigidBodyHandle {
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

impl_component_set_mut!(RigidBodyComponentsSet, dynamics::RigidBodyPosition,RigidBodyPosition, |data| &*data.1);
impl_component_set_mut!(RigidBodyComponentsSet, dynamics::RigidBodyVelocity,RigidBodyVelocity, |data| &*data.2);
impl_component_set_mut!(RigidBodyComponentsSet, dynamics::RigidBodyMassProps,RigidBodyMassProps, |data| &*data.3);
impl_component_set_mut!(RigidBodyComponentsSet, dynamics::RigidBodyIds,RigidBodyIds, |data| &*data.4);
impl_component_set_mut!(RigidBodyComponentsSet, dynamics::RigidBodyForces,RigidBodyForces, |data| &*data.5);
impl_component_set_mut!(RigidBodyComponentsSet, dynamics::RigidBodyCcd,RigidBodyCcd, |data| &*data.6);
impl_component_set_mut!(RigidBodyComponentsSet, dynamics::RigidBodyColliders,RigidBodyColliders, |data| &*data.7);
impl_component_set_mut!(RigidBodyComponentsSet, dynamics::RigidBodyDamping,RigidBodyDamping, |data| &*data.8);
impl_component_set_mut!(RigidBodyComponentsSet, dynamics::RigidBodyDominance,RigidBodyDominance, |data| &*data.9);
impl_component_set_mut!(RigidBodyComponentsSet, dynamics::RigidBodyType,RigidBodyType, |data| &*data.10);
impl_component_set_mut!(RigidBodyComponentsSet, dynamics::RigidBodyChanges,RigidBodyChanges, |data| &*data.11);
impl_component_set_mut!(RigidBodyComponentsSet, dynamics::RigidBodyActivation,RigidBodyActivation, |data| &*data
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
            body_type: RigidBodyType(rapier2d::prelude::RigidBodyType::Dynamic),
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
