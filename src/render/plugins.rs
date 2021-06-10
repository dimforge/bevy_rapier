use crate::render::systems;
use bevy::prelude::*;

/// Plugin responsible for creating meshes to render the Rapier physics scene.
pub struct RapierRenderPlugin;

impl Plugin for RapierRenderPlugin {
    fn build(&self, app: &mut AppBuilder) {
        app.add_system_to_stage(
            CoreStage::PreUpdate,
            systems::create_collider_renders_system
                .system()
                .label(systems::RenderSystems::CreateColliderRenders),
        );
        app.add_system_to_stage(
            CoreStage::PreUpdate,
            systems::update_collider_render_mesh
                .system()
                .label(systems::RenderSystems::UpdateColliderRenderMesh),
        );
    }
}
