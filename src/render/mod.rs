use crate::plugin::RapierContext;
use bevy::prelude::*;
use bevy::transform::TransformSystem;
use rapier::math::{Point, Real};
use rapier::pipeline::{DebugRenderBackend, DebugRenderObject, DebugRenderPipeline};
pub use rapier::pipeline::{DebugRenderMode, DebugRenderStyle};
use std::fmt::Debug;

#[cfg(doc)]
use crate::prelude::Collider;

/// The color of a collider when using the debug-renderer.
///
/// Insert this component alongside the [`Collider`] component to
/// force to a specific value the color used to render the
/// collider.
#[derive(Copy, Clone, Component, PartialEq, Debug, Reflect)]
pub struct ColliderDebugColor(pub Hsla);

/// Plugin rensponsible for rendering (using lines) what Rapier "sees" when performing
/// its physics simulation. This is typically useful to check proper
/// alignment between colliders and your own visual assets.
pub struct RapierDebugRenderPlugin {
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
            style: DebugRenderStyle::default(),
            mode: DebugRenderMode::default(),
        }
    }
}

impl RapierDebugRenderPlugin {
    /// Initialize the render plugin such that it is initially disabled.
    pub fn disabled(mut self) -> Self {
        self.enabled = false;
        self
    }
}

/// Context to control some aspect of the debug-renderer after initialization.
#[derive(Resource, Reflect)]
#[reflect(Resource)]
pub struct DebugRenderContext {
    /// Is the debug-rendering currently enabled?
    pub enabled: bool,
    /// Pipeline responsible for rendering. Access `pipeline.mode` and `pipeline.style`
    /// to modify the set of rendered elements, and modify the default coloring rules.
    #[reflect(ignore)]
    pub pipeline: DebugRenderPipeline,
}

impl Default for DebugRenderContext {
    fn default() -> Self {
        Self {
            enabled: true,
            pipeline: DebugRenderPipeline::default(),
        }
    }
}

impl Plugin for RapierDebugRenderPlugin {
    fn build(&self, app: &mut App) {
        app.register_type::<DebugRenderContext>();

        app.insert_resource(DebugRenderContext {
            enabled: self.enabled,
            pipeline: DebugRenderPipeline::new(self.style, self.mode),
        })
        .add_systems(
            PostUpdate,
            debug_render_scene.after(TransformSystem::TransformPropagate),
        );
    }
}

struct BevyLinesRenderBackend<'world, 'state, 'world2, 'state2, 'a, 'b, 'c, 'p> {
    custom_colors: &'c Query<'world, 'state, &'a ColliderDebugColor>,
    context: &'b RapierContext,
    gizmos: &'p mut Gizmos<'world2, 'state2>,
}

impl<'world, 'state, 'world2, 'state2, 'a, 'b, 'c, 'p>
    BevyLinesRenderBackend<'world, 'state, 'world2, 'state2, 'a, 'b, 'c, 'p>
{
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

        color.map(|co: Hsla| co.to_f32_array()).unwrap_or(default)
    }
}

impl<'world, 'state, 'world2, 'state2, 'a, 'b, 'c, 'p> DebugRenderBackend
    for BevyLinesRenderBackend<'world, 'state, 'world2, 'state2, 'a, 'b, 'c, 'p>
{
    #[cfg(feature = "dim2")]
    fn draw_line(
        &mut self,
        object: DebugRenderObject,
        a: Point<Real>,
        b: Point<Real>,
        color: [f32; 4],
    ) {
        let color = self.object_color(object, color);
        self.gizmos.line(
            [a.x, a.y, 0.0].into(),
            [b.x, b.y, 0.0].into(),
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
        let color = self.object_color(object, color);
        self.gizmos.line(
            [a.x, a.y, a.z].into(),
            [b.x, b.y, b.z].into(),
            Color::hsla(color[0], color[1], color[2], color[3]),
        )
    }
}

fn debug_render_scene(
    rapier_context: Query<&RapierContext>,
    mut render_context: ResMut<DebugRenderContext>,
    mut gizmos: Gizmos,
    custom_colors: Query<&ColliderDebugColor>,
) {
    if !render_context.enabled {
        return;
    }
    for rapier_context in rapier_context.iter() {
        let mut backend = BevyLinesRenderBackend {
            custom_colors: &custom_colors,
            context: rapier_context,
            gizmos: &mut gizmos,
        };

        let unscaled_style = render_context.pipeline.style;
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
}
