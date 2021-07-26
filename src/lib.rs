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

pub extern crate nalgebra as na;
#[cfg(feature = "dim2")]
pub extern crate rapier2d as rapier;
#[cfg(feature = "dim3")]
pub extern crate rapier3d as rapier;

/// Plugins, resources, and components for physics simulation.
pub mod physics;
/// Plugins, resources, and components for debug rendering.
#[cfg(feature = "render")]
pub mod render;

pub mod prelude {
    pub use super::physics::{
        ColliderBundle, ColliderComponentsSet, ColliderPositionSync, IntoEntity, IntoHandle,
        JointBuilderComponent, NoUserData, PhysicsHooksWithQuery, PhysicsHooksWithQueryObject,
        QueryPipelineColliderComponentsQuery, QueryPipelineColliderComponentsSet,
        RapierConfiguration, RapierPhysicsPlugin, RigidBodyBundle, RigidBodyComponentsSet,
        RigidBodyPositionSync,
    };
    #[cfg(feature = "render")]
    pub use super::render::{ColliderDebugRender, RapierRenderPlugin};
    pub use rapier::prelude::*;
}
