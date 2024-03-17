use crate::geometry::CollisionGroups;
use bevy::prelude::Entity;

pub use rapier::pipeline::QueryFilter;
pub use rapier::pipeline::QueryFilterFlags;

impl<'a> From<CollisionGroups> for QueryFilter<'a> {
    fn from(groups: CollisionGroups) -> Self {
        Self::from(groups.0)
    }
}
