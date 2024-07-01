use bevy::prelude::*;
use rapier::dynamics::{ImpulseJointHandle, MultibodyJointHandle};

pub use rapier::dynamics::{JointAxesMask, JointAxis, MotorModel};

use super::{FixedJoint, GenericJoint, PrismaticJoint, RevoluteJoint, RopeJoint, SpringJoint};

#[cfg(feature = "dim3")]
use super::SphericalJoint;

/// Wrapper enum over a specific joint.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum JointDescription {
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

impl JointDescription {
    /// The underlying generic joint.
    pub fn generic_joint(&self) -> &GenericJoint {
        match self {
            JointDescription::FixedJoint(j) => &j.data,
            JointDescription::GenericJoint(j) => j,
            JointDescription::PrismaticJoint(j) => &j.data,
            JointDescription::RevoluteJoint(j) => j.data(),
            JointDescription::RopeJoint(j) => j.data(),
            #[cfg(feature = "dim3")]
            JointDescription::SphericalJoint(j) => j.data(),
            JointDescription::SpringJoint(j) => j.data(),
        }
    }
    /// The underlying generic joint.
    pub fn generic_joint_mut(&mut self) -> &mut GenericJoint {
        match self {
            JointDescription::FixedJoint(ref mut j) => &mut j.data,
            JointDescription::GenericJoint(ref mut j) => j,
            JointDescription::PrismaticJoint(ref mut j) => &mut j.data,
            JointDescription::RevoluteJoint(ref mut j) => &mut j.data,
            JointDescription::RopeJoint(ref mut j) => &mut j.data,
            #[cfg(feature = "dim3")]
            JointDescription::SphericalJoint(ref mut j) => &mut j.data,
            JointDescription::SpringJoint(ref mut j) => &mut j.data,
        }
    }
}

/// A trait used for different constraint types applied to joints,
/// see [`ImpulseJoint`] and [`MultibodyJoint`].
pub trait JointConstraint {
    /// The entity containing the rigid-body used as the first endpoint of this joint.
    fn parent(&self) -> Entity;
    /// Access the joint’s description.
    fn data(&self) -> &JointDescription;
    /// Access mutably the joint’s description.
    fn data_mut(&mut self) -> &mut JointDescription;
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
    pub data: JointDescription,
}

impl ImpulseJoint {
    /// Initializes an impulse-based joint from its first endpoint and the joint description.
    pub fn new(parent: Entity, data: impl Into<JointDescription>) -> Self {
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
    pub data: JointDescription,
}

impl MultibodyJoint {
    /// Initializes an joint based on reduced coordinates from its first endpoint and
    /// the joint description.
    pub fn new(parent: Entity, data: JointDescription) -> Self {
        Self { parent, data }
    }
}
impl JointConstraint for ImpulseJoint {
    fn parent(&self) -> Entity {
        self.parent
    }

    fn data(&self) -> &JointDescription {
        &self.data
    }

    fn data_mut(&mut self) -> &mut JointDescription {
        &mut self.data
    }
}
