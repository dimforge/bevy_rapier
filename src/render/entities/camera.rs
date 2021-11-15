use bevy::prelude::*;
use bevy::render::camera::{Camera, VisibleEntities};

/// Perspective Camera for use in the rapier_debug render pass.
/// Simple copy of PerspectiveCameraBundle but with the appropriate name for use in the
/// rapier_debug render pass.
/// **Note:** This is only needed when using the rapier_debug render pass
/// If you wish to only render with the main pass, this camera is not needed.
#[derive(Bundle, Debug)]
pub struct RapierDebugPerspectiveCameraBundle {
    pub camera: Camera,
    pub perspective_projection: bevy::render::camera::PerspectiveProjection,
    pub visible_entities: VisibleEntities,
    pub transform: Transform,
    pub global_transform: GlobalTransform,
}

impl Default for RapierDebugPerspectiveCameraBundle {
    fn default() -> Self {
        let PerspectiveCameraBundle {
            camera,
            perspective_projection,
            visible_entities,
            transform,
            global_transform,
        } = PerspectiveCameraBundle::with_name(&crate::render::render::CAMERA_RAPIER_DEBUG.to_string());
        Self {
            camera,
            perspective_projection,
            visible_entities,
            transform,
            global_transform,
        }
    }
}

/// Orthographic Camera for use in the rapier_debug render pass.
/// Simple copy of OrthographicCameraBundle but with the appropriate name for use in the
/// rapier_debug render pass.
/// **Note:** This is only needed when using the rapier_debug render pass
/// If you wish to only render with the main pass, this camera is not needed.
#[derive(Bundle, Debug)]
pub struct RapierDebugOrthographicCameraBundle {
    pub camera: Camera,
    pub orthographic_projection: bevy::render::camera::OrthographicProjection,
    pub visible_entities: VisibleEntities,
    pub transform: Transform,
    pub global_transform: GlobalTransform,
}

impl Default for RapierDebugOrthographicCameraBundle {
    fn default() -> Self {
        let OrthographicCameraBundle {
            camera,
            orthographic_projection,
            visible_entities,
            transform,
            global_transform,
        } = OrthographicCameraBundle::with_name(&crate::render::render::CAMERA_RAPIER_DEBUG.to_string());
        Self {
            camera,
            orthographic_projection,
            visible_entities,
            transform,
            global_transform,
        }
    }
}
