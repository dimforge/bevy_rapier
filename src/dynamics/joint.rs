use bevy_ecs::component::Component;
use bevy_ecs::entity::Entity;
use rapier::dynamics::{ImpulseJointHandle, MultibodyJointHandle};

pub use rapier::dynamics::{JointAxesMask, JointAxis, MotorModel};

use super::{FixedJoint, GenericJoint, PrismaticJoint, RevoluteJoint, RopeJoint, SpringJoint};

#[cfg(feature = "dim3")]
use super::SphericalJoint;

/// Wrapper enum over a specific joint.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum TypedJoint {
    /// See [`FixedJoint`]
    FixedJoint(FixedJoint),
    /// See [`GenericJoint`]
    GenericJoint(GenericJoint),
    /// See [`PrismaticJoint`]
    PrismaticJoint(PrismaticJoint),
    /// See [`RevoluteJoint`]
    RevoluteJoint(RevoluteJoint),
    /// See [`RopeJoint`]
    RopeJoint(RopeJoint),
    /// See [`SphericalJoint`]
    #[cfg(feature = "dim3")]
    SphericalJoint(SphericalJoint),
    /// See [`SpringJoint`]
    SpringJoint(SpringJoint),
}

impl AsMut<GenericJoint> for TypedJoint {
    fn as_mut(&mut self) -> &mut GenericJoint {
        match self {
            TypedJoint::FixedJoint(ref mut j) => &mut j.data,
            TypedJoint::GenericJoint(ref mut j) => j,
            TypedJoint::PrismaticJoint(ref mut j) => &mut j.data,
            TypedJoint::RevoluteJoint(ref mut j) => &mut j.data,
            TypedJoint::RopeJoint(ref mut j) => &mut j.data,
            #[cfg(feature = "dim3")]
            TypedJoint::SphericalJoint(ref mut j) => &mut j.data,
            TypedJoint::SpringJoint(ref mut j) => &mut j.data,
        }
    }
}

impl AsRef<GenericJoint> for TypedJoint {
    fn as_ref(&self) -> &GenericJoint {
        match self {
            TypedJoint::FixedJoint(j) => &j.data,
            TypedJoint::GenericJoint(j) => j,
            TypedJoint::PrismaticJoint(j) => &j.data,
            TypedJoint::RevoluteJoint(j) => &j.data,
            TypedJoint::RopeJoint(j) => &j.data,
            #[cfg(feature = "dim3")]
            TypedJoint::SphericalJoint(j) => &j.data,
            TypedJoint::SpringJoint(j) => &j.data,
        }
    }
}

/// The handle of an impulse joint added to the physics scene.
#[derive(Copy, Clone, Debug, Component)]
pub struct RapierImpulseJointHandle(pub ImpulseJointHandle);

/// The handle of a multibody joint added to the physics scene.
#[derive(Copy, Clone, Debug, Component)]
pub struct RapierMultibodyJointHandle(pub MultibodyJointHandle);

/// An impulse-based joint attached to two entities.
///
/// The first end-point of the joint is the rigid-body attached to
/// `ImpulseJoint::parent`. The second endpoint of the joint is the
/// rigid-body attached to the entity (or the parent of the entity)
/// containing this `ImpulseJoint` component.
///
/// To attach multiple impulse joints to the same rigid-body, multiple
/// joints can be added in the children of the entity containing that
/// rigid-body (this is similar to the technique used to attach multiple
/// colliders to the same rigid-body).
#[derive(Copy, Clone, Debug, PartialEq, Component)]
pub struct ImpulseJoint {
    /// The entity containing the rigid-body used as the first endpoint of this joint.
    pub parent: Entity,
    /// The joint’s description.
    pub data: TypedJoint,
}

impl ImpulseJoint {
    /// Initializes an impulse-based joint from its first endpoint and the joint description.
    pub fn new(parent: Entity, data: impl Into<TypedJoint>) -> Self {
        Self {
            parent,
            data: data.into(),
        }
    }
}

/// An joint based on generalized coordinates, attached to two entities.
///
/// The first end-point of the joint is the rigid-body attached to
/// `MultibodyJoint::parent`. The second endpoint of the joint is the
/// rigid-body attached to the entity containing this `MultibodyJoint` component.
///
/// Note that a set of multibody joints cannot form closed loops (for example a necklace).
/// If a closed loop is detected, the last joint that closes the loop is ignored, and an
/// error is printed to `stderr` (using `log::error!`).
#[derive(Copy, Clone, Debug, PartialEq, Component)]
pub struct MultibodyJoint {
    /// The entity containing the rigid-body used as the first endpoint of this joint.
    pub parent: Entity,
    /// The joint’s description.
    pub data: TypedJoint,
}

impl MultibodyJoint {
    /// Initializes an joint based on reduced coordinates from its first endpoint and
    /// the joint description.
    pub fn new(parent: Entity, data: TypedJoint) -> Self {
        Self { parent, data }
    }
}
