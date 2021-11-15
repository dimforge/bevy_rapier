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

#[derive(Debug)]
pub enum RapierDebugEntities {
    All,
    Colliders,
    Positions,
    Path
}

#[derive(Debug)]
pub struct RapierDebugToggleVisibility(pub RapierDebugEntities);

#[derive(Debug)]
pub struct RapierDebugToggleRenderPass(pub RapierDebugEntities);
