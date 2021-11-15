use bevy::prelude::*;
use bevy::render::pipeline::RenderPipeline;

use crate::render::render::PositionWireframeMaterial;

pub struct RapierDebugPositionSize(pub f32);

impl Default for RapierDebugPositionSize {
    fn default() -> RapierDebugPositionSize {
        #[cfg(feature = "dim3")]
        return RapierDebugPositionSize(0.1);
        #[cfg(feature = "dim2")]
        RapierDebugPositionSize(10.0)
    }
}

#[derive(Bundle)]
pub struct RapierDebugPositionBundle {
    pub mesh: Handle<Mesh>,
    pub size: RapierDebugPositionSize,
    pub material: Handle<PositionWireframeMaterial>,
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

impl Default for RapierDebugPositionBundle {
    fn default() -> Self {
        Self {
            render_pipelines: RenderPipelines::from_pipelines(vec![
                RenderPipeline::new(
                    crate::render::render::POSITION_PIPELINE_HANDLE.typed(),
                ),
            ]),
            visible: Visible { is_visible: true, is_transparent: true },
            mesh: Default::default(),
            size: Default::default(),
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
