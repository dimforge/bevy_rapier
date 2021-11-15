mod mesh;

pub mod render;
pub mod entities;
pub mod systems;

mod plugin;
pub use self::plugin::RapierDebugPlugin;

pub mod prelude {
    pub use crate::render::entities::*;
    pub use crate::render::{RapierDebugPlugin, RapierDebugToggleVisibility, RapierDebugToggleRenderPass, RapierDebugEntities};
}

/// Represents a set of debug entities or all of them.
#[derive(Debug)]
pub enum RapierDebugEntities {
    All,
    Colliders,
    Positions,
    Path
}

/// Event Used to toggle the visibility of a particular set of debug entities.
/// **Note:** This works by using the `bevy::render::draw::Visible` struct, it will make
/// any objects attached to the entity invisible as well.
#[derive(Debug)]
pub struct RapierDebugToggleVisibility(pub RapierDebugEntities);

/// Event used to modify the render pass a particular set of entities will appear on.
#[derive(Debug)]
pub struct RapierDebugToggleRenderPass(pub RapierDebugEntities);
