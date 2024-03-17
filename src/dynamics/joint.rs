use bevy::prelude::*;
use rapier::dynamics::GenericJoint;

pub use rapier::dynamics::{
    ImpulseJointHandle, JointAxesMask, JointAxis, MotorModel, MultibodyJointHandle,
};

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
    pub data: GenericJoint,
}

impl ImpulseJoint {
    /// Initializes an impulse-based joint from its first endpoint and the joint description.
    pub fn new(parent: Entity, data: impl Into<GenericJoint>) -> Self {
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
    pub data: GenericJoint,
}

impl MultibodyJoint {
    /// Initializes a joint based on reduced coordinates from its first endpoint and
    /// the joint description.
    pub fn new(parent: Entity, data: impl Into<GenericJoint>) -> Self {
        Self {
            parent,
            data: data.into(),
        }
    }
}

/// Indicates whether the impulse joint was created in the physics backend.
#[derive(Copy, Clone, Default, Debug, PartialEq, Eq, Component, Reflect)]
#[reflect(Component, PartialEq)]
pub struct ImpulseJointCreated;

/// Indicates whether the multibody joint was created in the physics backend.
#[derive(Copy, Clone, Default, Debug, PartialEq, Eq, Component, Reflect)]
#[reflect(Component, PartialEq)]
pub struct MultibodyJointCreated;
