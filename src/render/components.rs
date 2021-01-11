use bevy::prelude::*;
use rapier::geometry::ColliderHandle;

/// The marker component to render a collider shape.
pub struct DebugColliderShape {
    /// The desired render color of a Rapier collider.
    pub color: Option<Color>,
    /// The collider handle
    pub(in crate) collider_handle: Option<ColliderHandle>,
}

impl Default for DebugColliderShape {
    fn default() -> Self {
        Self {
            color: None,
            collider_handle: None,
        }
    }
}
