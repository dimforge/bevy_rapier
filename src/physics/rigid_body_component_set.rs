use super::{IntoEntity, IntoHandle};
use crate::physics::wrapper::{
    RigidBodyActivationComponent, RigidBodyCcdComponent, RigidBodyChangesComponent,
    RigidBodyCollidersComponent, RigidBodyDampingComponent, RigidBodyDominanceComponent,
    RigidBodyForcesComponent, RigidBodyIdsComponent, RigidBodyMassPropsComponent,
    RigidBodyPositionComponent, RigidBodyTypeComponent, RigidBodyVelocityComponent,
};
use bevy::prelude::*;
use rapier::data::{ComponentSet, ComponentSetMut, ComponentSetOption, Index};
use rapier::dynamics;
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
    &'a mut RigidBodyPositionComponent,
    &'a mut RigidBodyVelocityComponent,
    &'a mut RigidBodyMassPropsComponent,
    &'a mut RigidBodyIdsComponent,
    &'a mut RigidBodyForcesComponent,
    &'a mut RigidBodyCcdComponent,
    &'a mut RigidBodyCollidersComponent,
    &'a mut RigidBodyDampingComponent,
    &'a mut RigidBodyDominanceComponent,
    &'a mut RigidBodyTypeComponent,
    &'a mut RigidBodyChangesComponent,
    &'a mut RigidBodyActivationComponent,
);

pub type RigidBodyChangesQueryPayload<'a> = (
    Entity,
    &'a mut RigidBodyActivationComponent,
    &'a mut RigidBodyChangesComponent,
    Or<(
        Changed<RigidBodyPositionComponent>,
        Added<RigidBodyPositionComponent>,
    )>,
    Or<(
        Changed<RigidBodyTypeComponent>,
        Added<RigidBodyTypeComponent>,
    )>,
    Or<(
        Changed<RigidBodyCollidersComponent>,
        Added<RigidBodyCollidersComponent>,
    )>,
);

pub type RigidBodyChangesQueryFilter = (
    Or<(
        Changed<RigidBodyPositionComponent>,
        Added<RigidBodyPositionComponent>,
        Changed<RigidBodyVelocityComponent>,
        Added<RigidBodyVelocityComponent>,
        Changed<RigidBodyForcesComponent>,
        Added<RigidBodyForcesComponent>,
        Changed<RigidBodyActivationComponent>,
        Added<RigidBodyActivationComponent>,
        Changed<RigidBodyTypeComponent>,
        Added<RigidBodyTypeComponent>,
        Changed<RigidBodyCollidersComponent>,
        Added<RigidBodyCollidersComponent>,
    )>,
);

pub type RigidBodyComponentsQuerySet<'world, 'state, 'a> = ParamSet<
    'world,
    'state,
    (
        // Components query
        Query<'world, 'state, RigidBodyComponentsQueryPayload<'a>>,
        // Changes query
        Query<'world, 'state, RigidBodyChangesQueryPayload<'a>, RigidBodyChangesQueryFilter>,
    ),
>;

pub struct RigidBodyComponentsSet<'world, 'state, 'a>(
    pub Query<'world, 'state, RigidBodyComponentsQueryPayload<'a>>,
);

impl_component_set_mut!(
    RigidBodyComponentsSet,
    dynamics::RigidBodyPosition,
    RigidBodyPositionComponent,
    |data| &*data.1
);
impl_component_set_mut!(
    RigidBodyComponentsSet,
    dynamics::RigidBodyVelocity,
    RigidBodyVelocityComponent,
    |data| &*data.2
);
impl_component_set_mut!(
    RigidBodyComponentsSet,
    dynamics::RigidBodyMassProps,
    RigidBodyMassPropsComponent,
    |data| &*data.3
);
impl_component_set_mut!(
    RigidBodyComponentsSet,
    dynamics::RigidBodyIds,
    RigidBodyIdsComponent,
    |data| &*data.4
);
impl_component_set_mut!(
    RigidBodyComponentsSet,
    dynamics::RigidBodyForces,
    RigidBodyForcesComponent,
    |data| &*data.5
);
impl_component_set_mut!(
    RigidBodyComponentsSet,
    dynamics::RigidBodyCcd,
    RigidBodyCcdComponent,
    |data| &*data.6
);
impl_component_set_mut!(
    RigidBodyComponentsSet,
    dynamics::RigidBodyColliders,
    RigidBodyCollidersComponent,
    |data| &*data.7
);
impl_component_set_mut!(
    RigidBodyComponentsSet,
    dynamics::RigidBodyDamping,
    RigidBodyDampingComponent,
    |data| &*data.8
);
impl_component_set_mut!(
    RigidBodyComponentsSet,
    dynamics::RigidBodyDominance,
    RigidBodyDominanceComponent,
    |data| &*data.9
);
impl_component_set_mut!(
    RigidBodyComponentsSet,
    dynamics::RigidBodyType,
    RigidBodyTypeComponent,
    |data| &*data.10
);
impl_component_set_mut!(
    RigidBodyComponentsSet,
    dynamics::RigidBodyChanges,
    RigidBodyChangesComponent,
    |data| &*data.11
);
impl_component_set_mut!(
    RigidBodyComponentsSet,
    dynamics::RigidBodyActivation,
    RigidBodyActivationComponent,
    |data| &*data.12
);

#[derive(Bundle)]
pub struct RigidBodyBundle {
    pub body_type: RigidBodyTypeComponent,
    pub position: RigidBodyPositionComponent,
    pub velocity: RigidBodyVelocityComponent,
    pub mass_properties: RigidBodyMassPropsComponent,
    pub forces: RigidBodyForcesComponent,
    pub activation: RigidBodyActivationComponent,
    pub damping: RigidBodyDampingComponent,
    pub dominance: RigidBodyDominanceComponent,
    pub ccd: RigidBodyCcdComponent,
    pub changes: RigidBodyChangesComponent,
    pub ids: RigidBodyIdsComponent,
    pub colliders: RigidBodyCollidersComponent,
}

impl Default for RigidBodyBundle {
    fn default() -> Self {
        Self {
            body_type: RigidBodyTypeComponent(rapier::prelude::RigidBodyType::Dynamic),
            position: RigidBodyPositionComponent::default(),
            velocity: RigidBodyVelocityComponent::default(),
            mass_properties: RigidBodyMassPropsComponent::default(),
            forces: RigidBodyForcesComponent::default(),
            activation: RigidBodyActivationComponent::default(),
            damping: RigidBodyDampingComponent::default(),
            dominance: RigidBodyDominanceComponent::default(),
            ccd: RigidBodyCcdComponent::default(),
            changes: RigidBodyChangesComponent::default(),
            ids: RigidBodyIdsComponent::default(),
            colliders: RigidBodyCollidersComponent::default(),
        }
    }
}
