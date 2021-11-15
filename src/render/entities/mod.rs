use bevy::prelude::*;

mod collider;
pub use self::collider::RapierDebugColliderWireframeBundle;

mod path;
pub use self::path::RapierDebugPathWireframeBundle;

mod position;
pub use self::position::{RapierDebugPositionBundle, RapierDebugPositionSize};

mod camera;
pub use self::camera::{
    RapierDebugOrthographicCameraBundle,
    RapierDebugPerspectiveCameraBundle
};

pub struct RapierDebugColliderLoaded;
pub struct RapierDebugPositionLoaded;
pub struct RapierDebugPathLoaded(pub Entity);

pub struct RapierDebugRenderCollider;
pub struct RapierDebugRenderPosition;
pub struct RapierDebugRenderPath;

#[derive(Debug)]
pub struct RapierDebugCollider {
    pub color: Color
}

impl Default for RapierDebugCollider {
    fn default() -> RapierDebugCollider {
        RapierDebugCollider { color: Color::RED }
    }
}

impl RapierDebugCollider {

    pub fn new(color: Color) -> RapierDebugCollider {
        RapierDebugCollider { color }
    }
}

#[derive(Debug)]
pub struct RapierDebugPath {
    pub color: Color,
    pub length: usize,
    pub dashed: bool
}

impl Default for RapierDebugPath {
    fn default() -> RapierDebugPath {
        RapierDebugPath {
            color: Color::RED,
            length: 1000,
            dashed: false
        }
    }
}

#[derive(Debug)]
pub struct RapierDebugPosition {
    pub size: f32,
    pub x: Color,
    pub y: Color,
    pub z: Color
}

impl Default for RapierDebugPosition {
    fn default() -> RapierDebugPosition {
        RapierDebugPosition {
            #[cfg(feature = "dim3")]
            size: 0.2,
            #[cfg(feature = "dim2")]
            size: 10.0,
            x: Color::RED,
            y: Color::BLUE,
            z: Color::GREEN
        }
    }
}
