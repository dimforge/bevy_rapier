use crate::dynamics::GenericJoint;
use bevy::prelude::*;
use rapier::dynamics::{ImpulseJointHandle, MultibodyJointHandle};

pub use rapier::dynamics::{JointAxesMask, JointAxis, MotorModel};

/// A trait used for different constraint types applied to joints,
/// see [`ImpulseJoint`] and [`MultibodyJoint`].
pub trait JointConstraint<JointDescriptionType = GenericJoint>
where
    JointDescriptionType: JointDescription,
{
    /// The entity containing the rigid-body used as the first endpoint of this joint.
    fn parent(&self) -> Entity;
    /// Access the joint’s description.
    fn data(&self) -> &JointDescriptionType;
    /// Access mutably the joint’s description.
    fn data_mut(&mut self) -> &mut JointDescriptionType;
}

/// A trait used for different constraint description applied to joints,
///
// TECH: It's mostly used to avoid being able
// to pass a builder as a `JointDescriptionType` to `JointConstraint`
#[diagnostic::on_unimplemented(note = "If you are using a builder, call `.build()`.")]
pub trait JointDescription
where
    Self: Into<GenericJoint>,
{
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
pub struct ImpulseJoint<JointDescriptionType = GenericJoint>
where
    JointDescriptionType: JointDescription,
{
    /// The entity containing the rigid-body used as the first endpoint of this joint.
    pub parent: Entity,
    /// The joint’s description.
    pub data: JointDescriptionType,
}

impl<T: JointDescription> ImpulseJoint<T> {
    /// Initializes an impulse-based joint from its first endpoint and the joint description.
    pub fn new(parent: Entity, data: T) -> Self {
        Self { parent, data: data }
    }
}

impl<T> JointConstraint<T> for ImpulseJoint<T>
where
    T: JointDescription,
{
    fn parent(&self) -> Entity {
        self.parent
    }

    fn data(&self) -> &T {
        &self.data
    }

    fn data_mut(&mut self) -> &mut T {
        &mut self.data
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
pub struct MultibodyJoint<JointDescriptionType = GenericJoint>
where
    JointDescriptionType: JointDescription,
{
    /// The entity containing the rigid-body used as the first endpoint of this joint.
    pub parent: Entity,
    /// The joint’s description.
    pub data: JointDescriptionType,
}

impl<T> MultibodyJoint<T>
where
    T: JointDescription,
{
    /// Initializes an joint based on reduced coordinates from its first endpoint and
    /// the joint description.
    pub fn new(parent: Entity, data: T) -> Self {
        Self { parent, data: data }
    }
}

impl<T> JointConstraint<T> for MultibodyJoint<T>
where
    T: JointDescription,
{
    fn parent(&self) -> Entity {
        self.parent
    }

    fn data(&self) -> &T {
        &self.data
    }

    fn data_mut(&mut self) -> &mut T {
        &mut self.data
    }
}
