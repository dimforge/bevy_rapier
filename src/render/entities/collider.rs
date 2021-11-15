use bevy::prelude::*;
use bevy::render::pipeline::RenderPipeline;

use crate::render::render::WireframeMaterial;

/// Collider Bundle using the correct material to display a collider.
/// **NOTE**: This is intended for internal/advanced use only.
#[derive(Bundle)]
pub struct RapierDebugColliderWireframeBundle {
    pub mesh: Handle<Mesh>,
    pub material: Handle<WireframeMaterial>,
    #[cfg(feature = "default_main_pass")]
    pub main_pass: bevy::render::render_graph::base::MainPass,
    #[cfg(not(feature = "default_main_pass"))]
    pub debug_pass: crate::render::render::RapierDebugPass,
    pub draw: Draw,
    pub visible: Visible,
    pub render_pipelines: RenderPipelines,
    pub transform: Transform,
    pub global_transform: GlobalTransform,
}

impl Default for RapierDebugColliderWireframeBundle {
    fn default() -> Self {
        Self {
            render_pipelines: RenderPipelines::from_pipelines(vec![
                RenderPipeline::new(
                    crate::render::render::COLLIDER_PIPELINE_HANDLE.typed(),
                ),
            ]),
            visible: Visible { is_visible: true, is_transparent: true },
            mesh: Default::default(),
            material: Default::default(),
            #[cfg(feature = "default_main_pass")]
            main_pass: Default::default(),
            #[cfg(not(feature = "default_main_pass"))]
            debug_pass: Default::default(),
            draw: Default::default(),
            transform: Default::default(),
            global_transform: Default::default(),
        }
    }
}
