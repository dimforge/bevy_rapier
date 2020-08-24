//!
//! # Official of Rapier to the Bevy game engine
//!
//! Rapier is a set of two Rust crates `rapier2d` and `rapier3d` for efficient cross-platform
//! physics simulation. It target application include video games, animation, robotics, etc.
//!
//! The `bevy_rapier` projects implements two other crates `bevy_rapier2d` and `bevy_rapier3d` which
//! defines physics plugins for the Bevy game engine.
//!

#![deny(missing_docs)]

pub extern crate nalgebra as na;
#[cfg(feature = "dim2")]
pub extern crate rapier2d as rapier;
#[cfg(feature = "dim3")]
pub extern crate rapier3d as rapier;

/// Plugins, resources, and components for physics simulation.
pub mod physics;
/// Plugins, resources, and components for debug rendering.
pub mod render;
