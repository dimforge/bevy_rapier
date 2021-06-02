use super::{IntoEntity, IntoHandle};
use bevy::prelude::*;
use rapier::data::{ComponentSet, ComponentSetMut, ComponentSetOption, Index};
use rapier::prelude::*;

impl IntoHandle<ColliderHandle> for Entity {
    #[inline]
    fn handle(self) -> ColliderHandle {
        ColliderHandle::from_raw_parts(self.id(), self.generation())
    }
}

impl IntoEntity for ColliderHandle {
    #[inline]
    fn entity(self) -> Entity {
        self.0.entity()
    }
}

pub type QueryPipelineColliderComponentsQuery<'a, 'b> = Query<
    'a,
    (
        Entity,
        &'b ColliderPosition,
        &'b ColliderShape,
        &'b ColliderFlags,
    ),
>;

pub struct QueryPipelineColliderComponentsSet<'a, 'b, 'c>(
    pub &'c QueryPipelineColliderComponentsQuery<'a, 'b>,
);

impl_component_set_wo_query_set!(
    QueryPipelineColliderComponentsSet,
    ColliderPosition,
    |data| data.1
);
impl_component_set_wo_query_set!(QueryPipelineColliderComponentsSet, ColliderShape, |data| {
    data.2
});
impl_component_set_wo_query_set!(QueryPipelineColliderComponentsSet, ColliderFlags, |data| {
    data.3
});

pub struct ColliderComponentsSet<'a, 'b, 'c>(pub ColliderComponentsQuery<'a, 'b, 'c>);

pub type ColliderComponentsQuery<'a, 'b, 'c> = QuerySet<(
    Query<
        'a,
        (
            Entity,
            &'b ColliderChanges,
            &'b ColliderPosition,
            &'b ColliderBroadPhaseData,
            &'b ColliderShape,
            &'b ColliderType,
            &'b ColliderMaterial,
            &'b ColliderFlags,
            Option<&'b ColliderParent>,
        ),
    >,
    Query<
        'a,
        (
            Entity,
            &'c mut ColliderChanges,
            &'c mut ColliderPosition,
            &'c mut ColliderBroadPhaseData,
        ),
    >,
    Query<
        'a,
        (
            Entity,
            &'c mut ColliderChanges,
            Or<(Changed<ColliderPosition>, Added<ColliderPosition>)>,
            Or<(Changed<ColliderFlags>, Added<ColliderFlags>)>,
            Or<(Changed<ColliderShape>, Added<ColliderShape>)>,
            Or<(Changed<ColliderType>, Added<ColliderType>)>,
            Option<Or<(Changed<ColliderParent>, Added<ColliderParent>)>>,
        ),
        Or<(
            Changed<ColliderPosition>,
            Added<ColliderPosition>,
            Changed<ColliderFlags>,
            Added<ColliderFlags>,
            Changed<ColliderShape>,
            Added<ColliderShape>,
            Changed<ColliderType>,
            Added<ColliderType>,
            Changed<ColliderParent>,
            Added<ColliderParent>,
        )>,
    >,
)>;

impl_component_set_mut!(ColliderComponentsSet, ColliderChanges, |data| data.1);
impl_component_set_mut!(ColliderComponentsSet, ColliderPosition, |data| data.2);
impl_component_set_mut!(ColliderComponentsSet, ColliderBroadPhaseData, |d| d.3);

impl_component_set!(ColliderComponentsSet, ColliderShape, |data| data.4);
impl_component_set!(ColliderComponentsSet, ColliderType, |data| data.5);
impl_component_set!(ColliderComponentsSet, ColliderMaterial, |data| data.6);
impl_component_set!(ColliderComponentsSet, ColliderFlags, |data| data.7);

impl_component_set_option!(ColliderComponentsSet, ColliderParent);

#[derive(Bundle)]
pub struct ColliderBundle {
    pub collider_type: ColliderType,
    pub shape: ColliderShape,
    pub position: ColliderPosition,
    pub material: ColliderMaterial,
    pub flags: ColliderFlags,
    pub mass_properties: ColliderMassProps,
    pub changes: ColliderChanges,
    pub bf_data: ColliderBroadPhaseData,
}

impl Default for ColliderBundle {
    fn default() -> Self {
        Self {
            collider_type: ColliderType::Solid,
            shape: ColliderShape::ball(0.5),
            position: ColliderPosition::default(),
            material: ColliderMaterial::default(),
            flags: ColliderFlags::default(),
            mass_properties: ColliderMassProps::default(),
            changes: ColliderChanges::default(),
            bf_data: ColliderBroadPhaseData::default(),
        }
    }
}
