use bevy::prelude::*;
use rapier::prelude::{
    Ball, Capsule, ColliderHandle, DVector, FeatureId, InteractionGroups, Point, Ray, SharedShape,
    Vector, DIM,
};
#[cfg(feature = "dim3")]
use rapier::prelude::{Cone, Cylinder};

use super::shape_views::*;
use crate::dynamics::CoefficientCombineRule;
use crate::geometry::{PointProjection, RayIntersection, VHACDParameters};
use crate::math::{Real, Rot, Vect};

use bevy::render::mesh::{Indices, VertexAttributeValues};

#[derive(Copy, Clone, Debug, Component)]
pub struct RapierColliderHandle(pub ColliderHandle);

#[cfg(feature = "dim3")]
#[derive(Component, Debug, Clone)]
pub enum AsyncCollider {
    Mesh(Handle<Mesh>),
    ConvexDecomposition(Handle<Mesh>, VHACDParameters),
}

#[derive(Component, Clone)] // TODO: Reflect
pub struct Collider {
    pub raw: SharedShape,
    pub(crate) unscaled: SharedShape,
    pub(crate) scale: Vect,
}

impl Into<Collider> for SharedShape {
    fn into(self) -> Collider {
        Collider {
            raw: self.clone(),
            unscaled: self,
            scale: Vect::ONE,
        }
    }
}

#[derive(Copy, Clone, Debug, Component)] // TODO: Reflect
pub enum ColliderScale {
    /// This scale will be multiplied with the scale in the `Transform` component
    /// before being applied to the collider.
    Relative(Vect),
    /// This scale will replace the one specified in the `Transform` component.
    Absolute(Vect),
}

#[derive(Copy, Clone, Debug, Component, Reflect)]
pub struct Sensor;

#[derive(Copy, Clone, Debug, Component, Reflect)]
pub struct Density(pub f32);

impl Default for Density {
    fn default() -> Self {
        Self(1.0)
    }
}

#[derive(Copy, Clone, Debug, Component, Reflect)]
pub struct Friction {
    pub coefficient: f32,
    pub combine_rule: CoefficientCombineRule,
}

impl Default for Friction {
    fn default() -> Self {
        Self {
            coefficient: 0.5,
            combine_rule: CoefficientCombineRule::Average,
        }
    }
}

impl Friction {
    pub fn new(coefficient: f32) -> Self {
        Self {
            coefficient,
            ..Default::default()
        }
    }
}

#[derive(Copy, Clone, Debug, Component, Reflect)]
pub struct Restitution {
    pub coefficient: f32,
    pub combine_rule: CoefficientCombineRule,
}

impl Restitution {
    pub fn new(coefficient: f32) -> Self {
        Self {
            coefficient,
            ..Default::default()
        }
    }
}

impl Default for Restitution {
    fn default() -> Self {
        Self {
            coefficient: 0.0,
            combine_rule: CoefficientCombineRule::Average,
        }
    }
}

bitflags::bitflags! {
    #[derive(Component, Reflect)]
    #[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
    /// Flags affecting whether or not collision-detection happens between two colliders
    /// depending on the type of rigid-bodies they are attached to.
    pub struct ActiveCollisionTypes: u16 {
        /// Enable collision-detection between a collider attached to a dynamic body
        /// and another collider attached to a dynamic body.
        const DYNAMIC_DYNAMIC = 0b0000_0000_0000_0001;
        /// Enable collision-detection between a collider attached to a dynamic body
        /// and another collider attached to a kinematic body.
        const DYNAMIC_KINEMATIC = 0b0000_0000_0000_1100;
        /// Enable collision-detection between a collider attached to a dynamic body
        /// and another collider attached to a fixed body (or not attached to any body).
        const DYNAMIC_STATIC  = 0b0000_0000_0000_0010;
        /// Enable collision-detection between a collider attached to a kinematic body
        /// and another collider attached to a kinematic body.
        const KINEMATIC_KINEMATIC = 0b1100_1100_0000_0000;

        /// Enable collision-detection between a collider attached to a kinematic body
        /// and another collider attached to a fixed body (or not attached to any body).
        const KINEMATIC_STATIC = 0b0010_0010_0000_0000;

        /// Enable collision-detection between a collider attached to a fixed body (or
        /// not attached to any body) and another collider attached to a fixed body (or
        /// not attached to any body).
        const STATIC_STATIC = 0b0000_0000_0010_0000;
    }
}

impl Default for ActiveCollisionTypes {
    fn default() -> Self {
        Self::DYNAMIC_DYNAMIC | Self::DYNAMIC_KINEMATIC | Self::DYNAMIC_STATIC
    }
}

impl Into<rapier::geometry::ActiveCollisionTypes> for ActiveCollisionTypes {
    fn into(self) -> rapier::geometry::ActiveCollisionTypes {
        rapier::geometry::ActiveCollisionTypes::from_bits(self.bits)
            .expect("Internal error: invalid active events conversion.")
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, Component, Reflect)]
pub struct CollisionGroups {
    pub memberships: u32,
    pub filters: u32,
}

impl Default for CollisionGroups {
    fn default() -> Self {
        Self {
            memberships: u32::MAX,
            filters: u32::MAX,
        }
    }
}

impl Into<InteractionGroups> for CollisionGroups {
    fn into(self) -> InteractionGroups {
        InteractionGroups {
            memberships: self.memberships,
            filter: self.filters,
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, Component, Reflect)]
pub struct SolverGroups {
    pub memberships: u32,
    pub filters: u32,
}

impl Default for SolverGroups {
    fn default() -> Self {
        Self {
            memberships: u32::MAX,
            filters: u32::MAX,
        }
    }
}

impl Into<InteractionGroups> for SolverGroups {
    fn into(self) -> InteractionGroups {
        InteractionGroups {
            memberships: self.memberships,
            filter: self.filters,
        }
    }
}

bitflags::bitflags! {
    #[derive(Component, Reflect)]
    #[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
    /// Flags affecting the behavior of the constraints solver for a given contact manifold.
    pub struct ActiveHooks: u32 {
        /// If set, Rapier will call `PhysicsHooks::filter_contact_pair` whenever relevant.
        const FILTER_CONTACT_PAIRS = 0b0001;
        /// If set, Rapier will call `PhysicsHooks::filter_intersection_pair` whenever relevant.
        const FILTER_INTERSECTION_PAIR = 0b0010;
        /// If set, Rapier will call `PhysicsHooks::modify_solver_contact` whenever relevant.
        const MODIFY_SOLVER_CONTACTS = 0b0100;
    }
}

impl Into<rapier::pipeline::ActiveHooks> for ActiveHooks {
    fn into(self) -> rapier::pipeline::ActiveHooks {
        rapier::pipeline::ActiveHooks::from_bits(self.bits)
            .expect("Internal error: invalid active events conversion.")
    }
}

bitflags::bitflags! {
    #[derive(Component, Reflect)]
    #[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
    /// Flags affecting the events generated for this collider.
    pub struct ActiveEvents: u32 {
        /// If set, Rapier will call `EventHandler::handle_intersection_event` and
        /// `EventHandler::handle_contact_event` whenever relevant for this collider.
        const COLLISION_EVENTS = 0b0001;
    }
}

impl Into<rapier::pipeline::ActiveEvents> for ActiveEvents {
    fn into(self) -> rapier::pipeline::ActiveEvents {
        rapier::pipeline::ActiveEvents::from_bits(self.bits)
            .expect("Internal error: invalid active events conversion.")
    }
}
