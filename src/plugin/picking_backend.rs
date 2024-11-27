//! A picking backend for Rapier physics entities.
//!
//! By default, all colliders are pickable. Picking can be disabled for individual entities
//! by adding [`PickingBehavior::IGNORE`].
//!
//! To make mesh picking entirely opt-in, set [`RapierPickingSettings::require_markers`]
//! to `true` and add a [`RayCastPickable`] component to the desired camera and target entities.
//!
//! To manually perform mesh ray casts independent of picking, use [`QueryPipeline`].

use bevy::app::prelude::*;
use bevy::ecs::prelude::*;
use bevy::picking::{
    backend::{ray::RayMap, HitData, PointerHits},
    prelude::*,
    PickSet,
};
use bevy::reflect::prelude::*;
use bevy::render::{prelude::*, view::RenderLayers};

use super::RapierContext;

/// Runtime settings for the [`MeshPickingPlugin`].
#[derive(Resource, Reflect)]
#[reflect(Resource, Default)]
pub struct RapierPickingSettings {
    /// When set to `true` ray casting will only happen between cameras and entities marked with
    /// [`RayCastPickable`]. `false` by default.
    ///
    /// This setting is provided to give you fine-grained control over which cameras and entities
    /// should be used by the mesh picking backend at runtime.
    pub require_markers: bool,

    /// Determines how mesh picking should consider [`Visibility`]. When set to [`RayCastVisibility::Any`],
    /// ray casts can be performed against both visible and hidden entities.
    ///
    /// Defaults to [`RayCastVisibility::VisibleInView`], only performing picking against visible entities
    /// that are in the view of a camera.
    pub ray_cast_visibility: RayCastVisibility,
}

impl Default for RapierPickingSettings {
    fn default() -> Self {
        Self {
            require_markers: false,
            ray_cast_visibility: RayCastVisibility::VisibleInView,
        }
    }
}

/// An optional component that marks cameras and target entities that should be used in the [`MeshPickingPlugin`].
/// Only needed if [`MeshPickingSettings::require_markers`] is set to `true`, and ignored otherwise.
#[derive(Debug, Clone, Default, Component, Reflect)]
#[reflect(Component, Default)]
pub struct RayCastPickable;

/// Adds the mesh picking backend to your app.
#[derive(Clone, Default)]
pub struct RapierPickingPlugin;

impl Plugin for RapierPickingPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<MeshPickingSettings>()
            .register_type::<(RayCastPickable, RapierPickingSettings)>()
            .add_systems(PreUpdate, update_hits.in_set(PickSet::Backend));
    }
}

/// Casts rays into the scene using [`MeshPickingSettings`] and sends [`PointerHits`] events.
#[allow(clippy::too_many_arguments)]
pub fn update_hits(
    backend_settings: Res<MeshPickingSettings>,
    ray_map: Res<RayMap>,
    picking_cameras: Query<(&Camera, Option<&RayCastPickable>, Option<&RenderLayers>)>,
    pickables: Query<&PickingBehavior>,
    marked_targets: Query<&RayCastPickable>,
    layers: Query<&RenderLayers>,
    rapier_context: Query<&RapierContext>,
    mut output: EventWriter<PointerHits>,
) {
    for (&ray_id, &ray) in ray_map.map().iter() {
        let Ok((camera, cam_pickable, cam_layers)) = picking_cameras.get(ray_id.camera) else {
            continue;
        };
        if backend_settings.require_markers && cam_pickable.is_none() {
            continue;
        }
        let order = camera.order as f32;
        for rapier_context in rapier_context.iter() {
            let mut picks = Vec::new();
            #[cfg(feature = "dim2")]
            rapier_context.intersections_with_point(
                bevy::math::Vec2::new(ray.origin.x, ray.origin.y),
                crate::prelude::QueryFilter::default(),
                |entity| {
                    let hit_data = HitData {
                        camera: ray_id.camera,
                        position: Some(bevy::math::Vec3::new(ray.origin.x, ray.origin.y, 0.0)),
                        normal: None,
                        depth: 0.0,
                    };
                    picks.push((entity, hit_data));
                    true
                },
            );
            #[cfg(feature = "dim3")]
            rapier_context.intersections_with_ray(
                ray.origin,
                ray.direction.into(),
                f32::MAX,
                true,
                crate::prelude::QueryFilter::default(),
                |entity, intersection| {
                    let hit_data = HitData {
                        camera: ray_id.camera,
                        position: Some(intersection.point),
                        normal: Some(intersection.normal),
                        depth: intersection.time_of_impact,
                    };
                    picks.push((entity, hit_data));
                    true
                },
            );
            if !picks.is_empty() {
                output.send(PointerHits::new(ray_id.pointer, picks, order));
            }
        }
    }
}
