use crate::plugin::RapierContext;
use bevy::prelude::*;
use lines::DebugLines;
use rapier::math::{Point, Real, DIM};
use rapier::pipeline::{DebugRenderBackend, DebugRenderObject, DebugRenderPipeline};
pub use rapier::pipeline::{DebugRenderMode, DebugRenderStyle};
use std::fmt::Debug;

mod lines;

#[derive(Copy, Clone, Component, PartialEq, Debug)]
pub struct ColliderDebugColor(pub Color);

pub struct RapierDebugRenderPlugin {
    pub depth_test: bool,
    pub style: DebugRenderStyle,
    pub mode: DebugRenderMode,
}

impl Default for RapierDebugRenderPlugin {
    #[cfg(feature = "dim2")]
    fn default() -> Self {
        let mut style = DebugRenderStyle::default();
        style.rigid_body_axes_length = 20.0; // 20 pixels by default.

        Self {
            depth_test: cfg!(feature = "dim3"),
            style,
            mode: DebugRenderMode::all(),
        }
    }
    #[cfg(feature = "dim3")]
    fn default() -> Self {
        Self {
            depth_test: cfg!(feature = "dim3"),
            style: DebugRenderStyle::default(),
            mode: DebugRenderMode::all(),
        }
    }
}

#[derive(Default)]
pub struct DebugRenderContext {
    pub pipeline: DebugRenderPipeline,
}

impl Plugin for RapierDebugRenderPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugin(lines::DebugLinesPlugin::with_depth_test(self.depth_test))
            .insert_resource(DebugRenderContext {
                pipeline: DebugRenderPipeline::new(self.style.clone(), self.mode),
            })
            .add_system_to_stage(CoreStage::Update, debug_render_scene);
    }
}

struct BevyLinesRenderBackend<'world, 'state, 'a, 'b, 'c> {
    physics_scale: f32,
    custom_colors: Query<'world, 'state, &'a ColliderDebugColor>,
    context: &'b RapierContext,
    lines: &'c mut DebugLines,
}

impl<'world, 'state, 'a, 'b, 'c> BevyLinesRenderBackend<'world, 'state, 'a, 'b, 'c> {
    fn object_color(&self, object: DebugRenderObject, default: [f32; 4]) -> [f32; 4] {
        let color = match object {
            DebugRenderObject::Collider(h, ..) => self.context.colliders.get(h).and_then(|co| {
                self.custom_colors
                    .get(Entity::from_bits(co.user_data as u64))
                    .map(|co| co.0)
                    .ok()
            }),
            _ => None,
        };

        color.map(|co| co.as_hlsa_f32()).unwrap_or(default)
    }
}

impl<'world, 'state, 'a, 'b, 'c> DebugRenderBackend
    for BevyLinesRenderBackend<'world, 'state, 'a, 'b, 'c>
{
    #[cfg(feature = "dim2")]
    fn draw_line(
        &mut self,
        object: DebugRenderObject,
        a: Point<Real>,
        b: Point<Real>,
        color: [f32; 4],
    ) {
        let scale = self.physics_scale;
        let color = self.object_color(object, color);
        self.lines.line_colored(
            [a.x * scale, a.y * scale, 0.0].into(),
            [b.x * scale, b.y * scale, 0.0].into(),
            0.0,
            Color::hsla(color[0], color[1], color[2], color[3]),
        )
    }

    #[cfg(feature = "dim3")]
    fn draw_line(
        &mut self,
        object: DebugRenderObject,
        a: Point<Real>,
        b: Point<Real>,
        color: [f32; 4],
    ) {
        let scale = self.physics_scale;
        let color = self.object_color(object, color);
        self.lines.line_colored(
            [a.x * scale, a.y * scale, a.z * scale].into(),
            [b.x * scale, b.y * scale, b.z * scale].into(),
            0.0,
            Color::hsla(color[0], color[1], color[2], color[3]),
        )
    }
}

fn debug_render_scene(
    rapier_context: Res<RapierContext>,
    mut render_context: ResMut<DebugRenderContext>,
    mut lines: ResMut<DebugLines>,
    custom_colors: Query<&ColliderDebugColor>,
) {
    let mut backend = BevyLinesRenderBackend {
        physics_scale: rapier_context.physics_scale,
        custom_colors,
        context: &*rapier_context,
        lines: &mut *lines,
    };

    let unscaled_style = render_context.pipeline.style;
    render_context.pipeline.style.rigid_body_axes_length /= rapier_context.physics_scale;
    render_context.pipeline.render(
        &mut backend,
        &rapier_context.bodies,
        &rapier_context.colliders,
        &rapier_context.impulse_joints,
        &rapier_context.multibody_joints,
    );
    render_context.pipeline.style = unscaled_style;
}
