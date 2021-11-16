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

/// Used to mark colliders that have had the debug collider entity added as a child entity.
/// Can be used to query for all colliders that have a collider debug entity.
pub struct RapierDebugColliderLoaded;
/// Used to mark positions that have had the debug position entity added as a child entity.
/// Userful for quering all the positions that have a position debug entity.
pub struct RapierDebugPositionLoaded;
/// Used to mark path entties that have had the debug path entity added as a child.
/// Userful for quering all the path entities that have a path debug entity.
pub struct RapierDebugPathLoaded(pub Entity);

/// Marker component to find the debug collider entities.
pub struct RapierDebugRenderCollider;
/// Marker component to find the debug position entities.
pub struct RapierDebugRenderPosition;
/// Marker component to find the debug path entities.
pub struct RapierDebugRenderPath;

/// Renders the attached collider. It will attach a child entity with a
/// `RapierDebugColliderWireframeBundle` bundle.
/// **Note:** If the attached collider is a 'Solid' the outline will be solid, if it's a
/// 'Sensor' the outline will be made out of a dashed line.
#[derive(Debug)]
pub struct RapierDebugCollider {
    /// Color used as the collider outline. default: Color::RED
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

/// Renders the Path a Collider or RigidBody has followed over time.
/// This spawns a RapierDebugPathWireframeBundle as a root entity.
#[derive(Debug)]
pub struct RapierDebugPath {
    /// The color of the path. default: Color::RED
    pub color: Color,
    /// The total length of the path in points. default: 1000
    pub length: usize,
    /// dashed outline or solid. default: false
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

/// Gizmo That shows the position of a Collider or RigidBody.
#[derive(Debug)]
pub struct RapierDebugPosition {
    /// Size of all 3 of the gizmo axis's.: default: 3d: 0.2, 2d: 10.0
    pub size: f32,
    //// Color of the gizmo x axis. default: Color::RED
    pub x: Color,
    //// Color of the gizmo y axis. default: Color::BLUE
    pub y: Color,
    //// Color of the gizmo z axis. default: Color::GREEN
    pub z: Color
}

impl Default for RapierDebugPosition {
    fn default() -> RapierDebugPosition {
        RapierDebugPosition {
            #[cfg(feature = "dim3")]
            size: 0.2,
            #[cfg(feature = "dim2")]
            size: 0.5,
            x: Color::RED,
            y: Color::BLUE,
            z: Color::GREEN
        }
    }
}
