use crate::render::lines::DebugLinesConfig;
use crate::{plugin::RapierContext, render::lines::DrawLinesLabel};
use bevy::prelude::*;
use bevy::render::Render;
use lines::DebugLines;
use rapier::math::{Point, Real};
use rapier::pipeline::{DebugRenderBackend, DebugRenderObject, DebugRenderPipeline};
pub use rapier::pipeline::{DebugRenderMode, DebugRenderStyle};
use std::fmt::Debug;

mod lines;

/// The color of a collider when using the debug-renderer.
///
/// Insert this component alongside the collider component to
/// force to a specific value the color used to render the
/// collider.
#[derive(Copy, Clone, Component, PartialEq, Debug)]
pub struct ColliderDebugColor(pub Color);

/// Plugin rensponsible for rendering (using lines) what Rapier "sees" when performing
/// its physics simulation. This is typically useful to check proper
/// alignment between colliders and your own visual assets.
pub struct RapierDebugRenderPlugin {
    /// If set to `true`, depth-testing will be disabled when rendering,
    /// meaning that the debug-render lines will always appear on top
    /// of (won’t be occluded by) your own visual assets.
    pub always_on_top: bool,
    /// Is the debug-rendering enabled?
    pub enabled: bool,
    /// Control some aspects of the render coloring.
    pub style: DebugRenderStyle,
    /// Flags to select what part of physics scene is rendered (by default
    /// everything is rendered).
    pub mode: DebugRenderMode,
}

#[allow(clippy::derivable_impls)] // The 3D impl can be derived, but not the 2D impl.
impl Default for RapierDebugRenderPlugin {
    #[cfg(feature = "dim2")]
    fn default() -> Self {
        Self {
            enabled: true,
            always_on_top: true,
            style: DebugRenderStyle {
                rigid_body_axes_length: 20.0,
                ..Default::default()
            },
            mode: DebugRenderMode::default(),
        }
    }
    #[cfg(feature = "dim3")]
    fn default() -> Self {
        Self {
            enabled: true,
            always_on_top: false,
            style: DebugRenderStyle::default(),
            mode: DebugRenderMode::default(),
        }
    }
}

impl RapierDebugRenderPlugin {
    /// Initialize the render plugin such that its lines are always rendered on top of other objects.
    pub fn always_on_top(mut self) -> Self {
        self.always_on_top = true;
        self
    }

    /// Initialize the render plugin such that it is initially disabled.
    pub fn disabled(mut self) -> Self {
        self.enabled = false;
        self
    }
}

/// Context to control some aspect of the debug-renderer after initialization.
#[derive(Resource)]
pub struct DebugRenderContext {
    /// Is the debug-rendering currently enabled?
    pub enabled: bool,
    /// Pipeline responsible for rendering. Access `pipeline.mode` and `pipeline.style`
    /// to modify the set of rendered elements, and modify the default coloring rules.
    pub pipeline: DebugRenderPipeline,
    /// Are the debug-lines always rendered on top of other primitives?
    pub always_on_top: bool,
}

impl Default for DebugRenderContext {
    fn default() -> Self {
        Self {
            enabled: true,
            pipeline: DebugRenderPipeline::default(),
            always_on_top: true,
        }
    }
}

impl Plugin for RapierDebugRenderPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugin(lines::DebugLinesPlugin::always_on_top(self.always_on_top))
            .insert_resource(DebugRenderContext {
                enabled: self.enabled,
                pipeline: DebugRenderPipeline::new(self.style, self.mode),
                always_on_top: self.always_on_top,
            })
            .add_systems(PostUpdate, debug_render_scene.before(DrawLinesLabel));
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

        color.map(|co| co.as_hsla_f32()).unwrap_or(default)
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
    lines_config: ResMut<DebugLinesConfig>,
    mut lines: ResMut<DebugLines>,
    custom_colors: Query<&ColliderDebugColor>,
) {
    if !render_context.enabled {
        return;
    }

    *lines_config.always_on_top.write().unwrap() = render_context.always_on_top;
    let mut backend = BevyLinesRenderBackend {
        physics_scale: rapier_context.physics_scale,
        custom_colors,
        context: &rapier_context,
        lines: &mut lines,
    };

    let unscaled_style = render_context.pipeline.style;
    render_context.pipeline.style.rigid_body_axes_length /= rapier_context.physics_scale;
    render_context.pipeline.render(
        &mut backend,
        &rapier_context.bodies,
        &rapier_context.colliders,
        &rapier_context.impulse_joints,
        &rapier_context.multibody_joints,
        &rapier_context.narrow_phase,
    );
    render_context.pipeline.style = unscaled_style;
}
