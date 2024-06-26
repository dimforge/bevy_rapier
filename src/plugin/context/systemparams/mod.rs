mod rapier_context_access;

use bevy::{ecs::query::QueryData, prelude::Entity};
pub use rapier_context_access::*;

use super::RapierContextEntityLink;

#[derive(QueryData)]
pub struct RapierEntity {
    pub entity: Entity,
    pub rapier_context_link: &'static RapierContextEntityLink,
}
