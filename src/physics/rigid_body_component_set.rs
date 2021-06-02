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

pub type RigidBodyComponentsQuery<'a, 'b, 'c> = QuerySet<(
    Query<
        'a,
        (
            Entity,
            &'b RigidBodyPosition,
            &'b RigidBodyVelocity,
            &'b RigidBodyMassProps,
            &'b RigidBodyIds,
            &'b RigidBodyForces,
            &'b RigidBodyActivation,
            &'b RigidBodyChanges,
            &'b RigidBodyCcd,
            &'b RigidBodyColliders,
            &'b RigidBodyDamping,
            &'b RigidBodyDominance,
            &'b RigidBodyType,
        ),
    >,
    Query<
        'a,
        (
            Entity,
            &'c mut RigidBodyPosition,
            &'c mut RigidBodyVelocity,
            &'c mut RigidBodyMassProps,
            &'c mut RigidBodyIds,
            &'c mut RigidBodyForces,
            &'c mut RigidBodyActivation,
            &'c mut RigidBodyChanges,
            &'c mut RigidBodyCcd,
            // Need for handling collider removals.
            &'c mut RigidBodyColliders,
        ),
    >,
    Query<
        'a,
        (
            Entity,
            &'c mut RigidBodyChanges,
            &'c mut RigidBodyActivation,
            Or<(Changed<RigidBodyPosition>, Added<RigidBodyPosition>)>,
            Or<(Changed<RigidBodyVelocity>, Added<RigidBodyVelocity>)>,
            Or<(Changed<RigidBodyForces>, Added<RigidBodyForces>)>,
            Or<(Changed<RigidBodyType>, Added<RigidBodyType>)>,
            Or<(Changed<RigidBodyColliders>, Added<RigidBodyColliders>)>,
        ),
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
    >,
    Query<
        'a,
        &'c mut RigidBodyChanges,
        Or<(Changed<RigidBodyActivation>, Added<RigidBodyActivation>)>,
    >,
)>;

pub struct RigidBodyComponentsSet<'a, 'b, 'c>(pub RigidBodyComponentsQuery<'a, 'b, 'c>);

impl_component_set_mut!(RigidBodyComponentsSet, RigidBodyPosition, |data| data.1);
impl_component_set_mut!(RigidBodyComponentsSet, RigidBodyVelocity, |data| data.2);
impl_component_set_mut!(RigidBodyComponentsSet, RigidBodyMassProps, |data| data.3);
impl_component_set_mut!(RigidBodyComponentsSet, RigidBodyIds, |data| data.4);
impl_component_set_mut!(RigidBodyComponentsSet, RigidBodyForces, |data| data.5);
impl_component_set_mut!(RigidBodyComponentsSet, RigidBodyActivation, |data| data.6);
impl_component_set_mut!(RigidBodyComponentsSet, RigidBodyChanges, |data| data.7);
impl_component_set_mut!(RigidBodyComponentsSet, RigidBodyCcd, |data| data.8);
impl_component_set_mut!(RigidBodyComponentsSet, RigidBodyColliders, |data| data.9);

impl_component_set!(RigidBodyComponentsSet, RigidBodyDamping, |data| data.10);
impl_component_set!(RigidBodyComponentsSet, RigidBodyDominance, |data| data.11);
impl_component_set!(RigidBodyComponentsSet, RigidBodyType, |data| data.12);

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
