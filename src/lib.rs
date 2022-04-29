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

// #![deny(missing_docs)] // FIXME: deny this

#[macro_use]
#[cfg(feature = "serde-serialize")]
extern crate serde;

pub extern crate nalgebra as na;
#[cfg(feature = "dim2")]
pub extern crate rapier2d as rapier;
#[cfg(feature = "dim3")]
pub extern crate rapier3d as rapier;
pub use rapier::parry;

#[cfg(feature = "dim2")]
pub mod math {
    use bevy::math::Vec2;
    pub type Real = rapier::math::Real;
    pub type Vect = Vec2;
    pub type Rot = Real;
}

#[cfg(feature = "dim3")]
pub mod math {
    use bevy::math::{Quat, Vec3};
    pub type Real = rapier::math::Real;
    pub type Vect = Vec3;
    pub type Rot = Quat;
}

pub mod dynamics;
pub mod geometry;
pub mod pipeline;
pub mod plugin;

#[cfg(feature = "debug-render")]
pub mod render;
pub mod utils;

pub mod prelude {
    pub use crate::dynamics::*;
    pub use crate::geometry::*;
    pub use crate::math::*;
    pub use crate::pipeline::*;
    pub use crate::plugin::*;
    #[cfg(feature = "debug-render")]
    pub use crate::render::*;
}
