//!
//! # Official integration of Rapier to the Bevy game engine
//!
//! Rapier is a set of two Rust crates `rapier2d` and `rapier3d` for efficient cross-platform
//! physics simulation. It target application include video games, animation, robotics, etc.
//!
//! The `bevy_rapier` projects implements two other crates `bevy_rapier2d` and `bevy_rapier3d` which
//! defines physics plugins for the Bevy game engine.
//!
//! User documentation for `bevy_rapier` is on [the official Rapier site](https://rapier.rs/docs/).
//!

#![warn(missing_docs)]

#[macro_use]
#[cfg(feature = "serde-serialize")]
extern crate serde;

pub extern crate nalgebra as na;
#[cfg(feature = "dim2")]
pub extern crate rapier2d as rapier;
#[cfg(feature = "dim3")]
pub extern crate rapier3d as rapier;
pub use rapier::parry;

/// Type aliases to select the right vector/rotation types based
/// on the dimension used by the engine.
#[cfg(feature = "dim2")]
pub mod math {
    use bevy::math::Vec2;
    /// The real type (f32 or f64).
    pub type Real = rapier::math::Real;
    /// The vector type.
    pub type Vect = Vec2;
    /// The rotation type (in 2D this is an angle in radians).
    pub type Rot = Real;
}

/// Type aliases to select the right vector/rotation types based
/// on the dimension used by the engine.
#[cfg(feature = "dim3")]
pub mod math {
    use bevy::math::{Quat, Vec3};
    /// The real type (f32 or f64).
    pub type Real = rapier::math::Real;
    /// The vector type.
    pub type Vect = Vec3;
    /// The rotation type.
    pub type Rot = Quat;
}

/// Components related to physics dynamics (rigid-bodies, velocities, etc.)
pub mod dynamics;
/// Components related to physics geometry (colliders, collision-groups, etc.)
pub mod geometry;
/// Components and resources related to the physics simulation workflow (events, hooks, etc.)
pub mod pipeline;
/// The physics plugin and systems.
pub mod plugin;

/// The debug-renderer.
#[cfg(feature = "debug-render")]
pub mod render;
/// Miscellaneous helper functions.
pub mod utils;

/// Groups the most often used types.
pub mod prelude {
    pub use crate::dynamics::*;
    pub use crate::geometry::*;
    pub use crate::math::*;
    pub use crate::pipeline::*;
    pub use crate::plugin::*;
    #[cfg(feature = "debug-render")]
    pub use crate::render::*;
}
