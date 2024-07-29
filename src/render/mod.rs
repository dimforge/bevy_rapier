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
#[derive(Copy, Clone, Component, PartialEq, Debug)]
pub struct ColliderDebugColor(pub Hsla);

/// Overrides the global [`DebugRenderContext`] for a single collider.
#[derive(Copy, Clone, Reflect, Component, Eq, PartialEq, Default, Debug)]
pub enum ColliderDebug {
    /// Always render the debug gizmos for this collider, regardless of global config.
    #[default]
    AlwaysRender,
    /// Never render the debug gizmos for this collider, regardless of global config.
    NeverRender,
}

/// Plugin rensponsible for rendering (using lines) what Rapier "sees" when performing
/// its physics simulation. This is typically useful to check proper
/// alignment between colliders and your own visual assets.
pub struct RapierDebugRenderPlugin {
    /// Whether to show debug gizmos for all colliders.
    ///
    /// Can be overridden for individual colliders by adding a [`ColliderDebug`] component.
    pub default_collider_debug: ColliderDebug,
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
            default_collider_debug: ColliderDebug::AlwaysRender,
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
            default_collider_debug: ColliderDebug::AlwaysRender,
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
    /// Whether to show debug gizmos for all colliders.
    ///
    /// Can be overriden for individual colliders by adding a [`ColliderDebug`] component.
    pub default_collider_debug: ColliderDebug,
    /// Pipeline responsible for rendering. Access `pipeline.mode` and `pipeline.style`
    /// to modify the set of rendered elements, and modify the default coloring rules.
    #[reflect(ignore)]
    pub pipeline: DebugRenderPipeline,
}

impl Default for DebugRenderContext {
    fn default() -> Self {
        Self {
            enabled: true,
            default_collider_debug: ColliderDebug::AlwaysRender,
            pipeline: DebugRenderPipeline::default(),
        }
    }
}

impl Plugin for RapierDebugRenderPlugin {
    fn build(&self, app: &mut App) {
        app.register_type::<DebugRenderContext>();
        app.register_type::<ColliderDebug>();

        app.insert_resource(DebugRenderContext {
            enabled: self.enabled,
            default_collider_debug: self.default_collider_debug,
            pipeline: DebugRenderPipeline::new(self.style, self.mode),
        })
        .add_systems(
            PostUpdate,
            debug_render_scene.after(TransformSystem::TransformPropagate),
        );
    }
}

struct BevyLinesRenderBackend<'world, 'state, 'a, 'b> {
    custom_colors: Query<'world, 'state, &'a ColliderDebugColor>,
    default_collider_debug: ColliderDebug,
    override_visibility: Query<'world, 'state, &'a ColliderDebug>,
    context: &'b RapierContext,
    gizmos: Gizmos<'world, 'state>,
}

impl<'world, 'state, 'a, 'b> BevyLinesRenderBackend<'world, 'state, 'a, 'b> {
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

    fn drawing_enabled(&self, object: DebugRenderObject) -> bool {
        match object {
            DebugRenderObject::Collider(h, ..) => {
                let Some(collider) = self.context.colliders.get(h) else {
                    return false;
                };
                let entity = Entity::from_bits(collider.user_data as u64);

                let collider_debug =
                    if let Ok(collider_override) = self.override_visibility.get(entity) {
                        *collider_override
                    } else {
                        self.default_collider_debug
                    };
                collider_debug == ColliderDebug::AlwaysRender
            }
            _ => true,
        }
    }
}

impl<'world, 'state, 'a, 'b> DebugRenderBackend for BevyLinesRenderBackend<'world, 'state, 'a, 'b> {
    #[cfg(feature = "dim2")]
    fn draw_line(
        &mut self,
        object: DebugRenderObject,
        a: Point<Real>,
        b: Point<Real>,
        color: [f32; 4],
    ) {
        if !self.drawing_enabled(object) {
            return;
        }

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
        if !self.drawing_enabled(object) {
            return;
        }

        let color = self.object_color(object, color);
        self.gizmos.line(
            [a.x, a.y, a.z].into(),
            [b.x, b.y, b.z].into(),
            Color::hsla(color[0], color[1], color[2], color[3]),
        )
    }
}

fn debug_render_scene<'a>(
    rapier_context: Res<RapierContext>,
    mut render_context: ResMut<DebugRenderContext>,
    gizmos: Gizmos,
    custom_colors: Query<&'a ColliderDebugColor>,
    override_visibility: Query<&'a ColliderDebug>,
) {
    if !render_context.enabled {
        return;
    }

    let mut backend = BevyLinesRenderBackend {
        custom_colors,
        default_collider_debug: render_context.default_collider_debug,
        override_visibility,
        context: &rapier_context,
        gizmos,
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
