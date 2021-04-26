use super::{BundleBuilder, IntoEntity, IntoHandle};
use bevy::prelude::*;
use rapier::data::{ComponentSet, ComponentSetMut, ComponentSetOption, Index};
use rapier::geometry::{
    ColliderBroadPhaseData, ColliderChanges, ColliderGroups, ColliderHandle,
    ColliderMassProperties, ColliderMaterial, ColliderParent, ColliderPosition, ColliderShape,
    ColliderType,
};

impl IntoHandle<ColliderHandle> for Entity {
    fn handle(self) -> ColliderHandle {
        ColliderHandle::from_raw_parts(self.id(), self.generation())
    }
}

impl IntoEntity for ColliderHandle {
    fn entity(self) -> Entity {
        self.0.entity()
    }
}

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
            &'b ColliderGroups,
            &'b ColliderMaterial,
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
            Or<(Changed<ColliderGroups>, Added<ColliderGroups>)>,
            Or<(Changed<ColliderShape>, Added<ColliderShape>)>,
            Or<(Changed<ColliderType>, Added<ColliderType>)>,
            Option<Or<(Changed<ColliderParent>, Added<ColliderParent>)>>,
        ),
        Or<(
            Changed<ColliderPosition>,
            Added<ColliderPosition>,
            Changed<ColliderGroups>,
            Added<ColliderGroups>,
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
impl_component_set!(ColliderComponentsSet, ColliderGroups, |data| data.6);
impl_component_set!(ColliderComponentsSet, ColliderMaterial, |data| data.7);

impl_component_set_option!(ColliderComponentsSet, ColliderParent);

#[derive(Bundle)]
pub struct ColliderBundle {
    pub collider_type: ColliderType,
    pub shape: ColliderShape,
    pub position: ColliderPosition,
    pub material: ColliderMaterial,
    pub groups: ColliderGroups,
    pub mass_properties: ColliderMassProperties,
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
            groups: ColliderGroups::default(),
            mass_properties: ColliderMassProperties::default(),
            changes: ColliderChanges::default(),
            bf_data: ColliderBroadPhaseData::default(),
        }
    }
}
