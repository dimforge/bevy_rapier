mod rapier_context_systemparam;

use bevy::{ecs::query::QueryData, prelude::Entity};
pub use rapier_context_systemparam::*;

use super::RapierContextEntityLink;

#[derive(QueryData)]
pub(crate) struct RapierEntity {
    pub entity: Entity,
    pub rapier_context_link: &'static RapierContextEntityLink,
}
