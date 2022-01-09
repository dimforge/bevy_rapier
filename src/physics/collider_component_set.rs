use super::{IntoEntity, IntoHandle};
use crate::physics::wrapper::{
    ColliderBroadPhaseDataComponent, ColliderChangesComponent, ColliderFlagsComponent,
    ColliderMassPropsComponent, ColliderMaterialComponent, ColliderParentComponent,
    ColliderPositionComponent, ColliderShapeComponent, ColliderTypeComponent,
};
use bevy::prelude::*;
use rapier::data::{ComponentSet, ComponentSetMut, ComponentSetOption, Index};
use rapier::geometry;
impl IntoHandle<geometry::ColliderHandle> for Entity {
    #[inline]
    fn handle(self) -> geometry::ColliderHandle {
        geometry::ColliderHandle::from_raw_parts(self.id(), self.generation())
    }
}

impl IntoEntity for geometry::ColliderHandle {
    #[inline]
    fn entity(self) -> Entity {
        self.0.entity()
    }
}

pub type QueryPipelineColliderComponentsQuery<'world, 'state, 'a> = Query<
    'world,
    'state,
    (
        Entity,
        &'a ColliderPositionComponent,
        &'a ColliderShapeComponent,
        &'a ColliderFlagsComponent,
    ),
>;

pub struct QueryPipelineColliderComponentsSet<'world, 'state, 'a, 'c>(
    pub &'c QueryPipelineColliderComponentsQuery<'world, 'state, 'a>,
);

impl_component_set!(
    QueryPipelineColliderComponentsSet,
    geometry::ColliderPosition,
    ColliderPositionComponent,
    |data| data.1
);
impl_component_set!(
    QueryPipelineColliderComponentsSet,
    geometry::ColliderShape,
    ColliderShapeComponent,
    |data| data.2
);
impl_component_set!(
    QueryPipelineColliderComponentsSet,
    geometry::ColliderFlags,
    ColliderFlagsComponent,
    |data| data.3
);

pub struct ColliderComponentsSet<'world, 'state, 'a>(
    pub Query<'world, 'state, ColliderComponentsQueryPayload<'a>>,
);

pub type ColliderComponentsQueryPayload<'a> = (
    Entity,
    &'a mut ColliderChangesComponent,
    &'a mut ColliderPositionComponent,
    &'a mut ColliderBroadPhaseDataComponent,
    &'a mut ColliderShapeComponent,
    &'a mut ColliderTypeComponent,
    &'a mut ColliderMaterialComponent,
    &'a mut ColliderFlagsComponent,
    Option<&'a ColliderParentComponent>,
);

pub type ColliderChangesQueryPayload<'a> = (
    Entity,
    &'a mut ColliderChangesComponent,
    Or<(
        Changed<ColliderPositionComponent>,
        Added<ColliderPositionComponent>,
    )>,
    Or<(
        Changed<ColliderFlagsComponent>,
        Added<ColliderFlagsComponent>,
    )>,
    Or<(
        Changed<ColliderShapeComponent>,
        Added<ColliderShapeComponent>,
    )>,
    Or<(Changed<ColliderTypeComponent>, Added<ColliderTypeComponent>)>,
    Option<
        Or<(
            Changed<ColliderParentComponent>,
            Added<ColliderParentComponent>,
        )>,
    >,
);

pub type ColliderChangesQueryFilter = (
    Or<(
        Changed<ColliderPositionComponent>,
        Added<ColliderPositionComponent>,
        Changed<ColliderFlagsComponent>,
        Added<ColliderFlagsComponent>,
        Changed<ColliderShapeComponent>,
        Added<ColliderShapeComponent>,
        Changed<ColliderTypeComponent>,
        Added<ColliderTypeComponent>,
        Changed<ColliderParentComponent>,
        Added<ColliderParentComponent>,
    )>,
);

pub type ColliderComponentsQuerySet<'world, 'state, 'a> = QuerySet<
    'world,
    'state,
    (
        // Components query
        QueryState<ColliderComponentsQueryPayload<'a>>,
        // Changes query
        QueryState<ColliderChangesQueryPayload<'a>, ColliderChangesQueryFilter>,
    ),
>;

impl_component_set_mut!(
    ColliderComponentsSet,
    geometry::ColliderChanges,
    ColliderChangesComponent,
    |data| &*data.1
);
impl_component_set_mut!(
    ColliderComponentsSet,
    geometry::ColliderPosition,
    ColliderPositionComponent,
    |data| &*data.2
);
impl_component_set_mut!(
    ColliderComponentsSet,
    geometry::ColliderBroadPhaseData,
    ColliderBroadPhaseDataComponent,
    |d| &*d.3
);
impl_component_set_mut!(
    ColliderComponentsSet,
    geometry::ColliderShape,
    ColliderShapeComponent,
    |data| &*data.4
);
impl_component_set_mut!(
    ColliderComponentsSet,
    geometry::ColliderType,
    ColliderTypeComponent,
    |data| &*data.5
);
impl_component_set_mut!(
    ColliderComponentsSet,
    geometry::ColliderMaterial,
    ColliderMaterialComponent,
    |data| &*data.6
);
impl_component_set_mut!(
    ColliderComponentsSet,
    geometry::ColliderFlags,
    ColliderFlagsComponent,
    |data| &*data.7
);
impl_component_set_option!(
    ColliderComponentsSet,
    geometry::ColliderParent,
    ColliderParentComponent
);

#[derive(Bundle)]
pub struct ColliderBundle {
    pub collider_type: ColliderTypeComponent,
    pub shape: ColliderShapeComponent,
    pub position: ColliderPositionComponent,
    pub material: ColliderMaterialComponent,
    pub flags: ColliderFlagsComponent,
    pub mass_properties: ColliderMassPropsComponent,
    pub changes: ColliderChangesComponent,
    pub bf_data: ColliderBroadPhaseDataComponent,
}

impl Default for ColliderBundle {
    fn default() -> Self {
        Self {
            collider_type: ColliderTypeComponent(geometry::ColliderType::Solid),
            shape: ColliderShapeComponent(geometry::ColliderShape::ball(0.5)),
            position: ColliderPositionComponent::default(),
            material: ColliderMaterialComponent::default(),
            flags: ColliderFlagsComponent::default(),
            mass_properties: ColliderMassPropsComponent::default(),
            changes: ColliderChangesComponent::default(),
            bf_data: ColliderBroadPhaseDataComponent::default(),
        }
    }
}
