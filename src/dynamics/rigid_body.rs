use crate::math::Vect;
use bevy::prelude::*;
use rapier::prelude::{Isometry, RigidBodyType};
use std::ops::{Add, AddAssign, Sub, SubAssign};

pub use rapier::dynamics::{
    AdditionalMassProperties, Ccd, Damping, Dominance, LockedAxes, MassProperties, RigidBodyHandle,
    SleepState, Velocity,
};

/// A [`RigidBody`].
///
/// Related components:
/// - [`GlobalTransform`]: used as the ground truth for the bodies position.
/// - [`Velocity`]
/// - [`ExternalImpulse`]
/// - [`ExternalForce`]
/// - [`AdditionalMassProperties`]
/// - [`ReadMassProperties`]
/// - [`Damping`]
/// - [`Dominance`]
/// - [`Ccd`]: Helps prevent tunneling through thin objects or rigid bodies
///            moving at high velocities.
/// - [`LockedAxes`]
/// - [`RigidBodyDisabled`]
/// - [`GravityScale`]
#[derive(Copy, Clone, Debug, PartialEq, Eq, Component, Reflect, Default)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[reflect(Component, PartialEq)]
pub enum RigidBody {
    /// A `RigidBody::Dynamic` body can be affected by all external forces.
    #[default]
    Dynamic,
    /// A `RigidBody::Fixed` body cannot be affected by external forces.
    Fixed,
    /// A `RigidBody::KinematicPositionBased` body cannot be affected by any external forces but can be controlled
    /// by the user at the position level while keeping realistic one-way interaction with dynamic bodies.
    ///
    /// One-way interaction means that a kinematic body can push a dynamic body, but a kinematic body
    /// cannot be pushed by anything. In other words, the trajectory of a kinematic body can only be
    /// modified by the user and is independent of any contact or joint it is involved in.
    KinematicPositionBased,
    /// A `RigidBody::KinematicVelocityBased` body cannot be affected by any external forces but can be controlled
    /// by the user at the velocity level while keeping realistic one-way interaction with dynamic bodies.
    ///
    /// One-way interaction means that a kinematic body can push a dynamic body, but a kinematic body
    /// cannot be pushed by anything. In other words, the trajectory of a kinematic body can only be
    /// modified by the user and is independent of any contact or joint it is involved in.
    KinematicVelocityBased,
}

impl From<RigidBody> for RigidBodyType {
    fn from(rigid_body: RigidBody) -> RigidBodyType {
        match rigid_body {
            RigidBody::Dynamic => RigidBodyType::Dynamic,
            RigidBody::Fixed => RigidBodyType::Fixed,
            RigidBody::KinematicPositionBased => RigidBodyType::KinematicPositionBased,
            RigidBody::KinematicVelocityBased => RigidBodyType::KinematicVelocityBased,
        }
    }
}

impl From<RigidBodyType> for RigidBody {
    fn from(rigid_body: RigidBodyType) -> RigidBody {
        match rigid_body {
            RigidBodyType::Dynamic => RigidBody::Dynamic,
            RigidBodyType::Fixed => RigidBody::Fixed,
            RigidBodyType::KinematicPositionBased => RigidBody::KinematicPositionBased,
            RigidBodyType::KinematicVelocityBased => RigidBody::KinematicVelocityBased,
        }
    }
}

/// Center-of-mass, mass, and angular inertia.
///
/// When this is used as a component, this lets you read the total mass properties of
/// a [`RigidBody`] (including the colliders contribution). Modifying this component won’t
/// affect the mass-properties of the [`RigidBody`] (the attached colliders’ `ColliderMassProperties`
/// and the `AdditionalMassProperties` should be modified instead).
///
/// This only reads the mass from entities with a [`RigidBody`] component.
#[derive(Copy, Clone, Debug, Default, PartialEq, Component, Reflect)]
#[reflect(Component, PartialEq)]
pub struct ReadMassProperties(MassProperties);

impl ReadMassProperties {
    /// Get the [`MassProperties`] of this [`RigidBody`].
    pub fn get(&self) -> &MassProperties {
        &self.0
    }

    pub(crate) fn set(&mut self, mass_props: MassProperties) {
        self.0 = mass_props;
    }
}

impl std::ops::Deref for ReadMassProperties {
    type Target = MassProperties;
    fn deref(&self) -> &Self::Target {
        self.get()
    }
}

/// Entity that likely had their mass properties changed this frame.
#[derive(Deref, Copy, Clone, Debug, PartialEq, Event)]
pub struct MassModifiedEvent(pub Entity);

impl From<Entity> for MassModifiedEvent {
    fn from(entity: Entity) -> Self {
        Self(entity)
    }
}

/// Constant external forces applied continuously to a [`RigidBody`].
///
/// This force is applied at each timestep.
#[derive(Copy, Clone, Debug, Default, PartialEq, Component, Reflect)]
#[reflect(Component, PartialEq)]
pub struct ExternalForce {
    /// The linear force applied to the [`RigidBody`].
    pub force: Vect,
    /// The angular torque applied to the [`RigidBody`].
    #[cfg(feature = "dim2")]
    pub torque: f32,
    /// The angular torque applied to the [`RigidBody`].
    #[cfg(feature = "dim3")]
    pub torque: Vect,
}

impl ExternalForce {
    /// A force applied at a specific world-space point of a [`RigidBody`].
    ///
    /// # Parameters
    /// - `force`: the force to apply.
    /// - `point`: the point (world-space) where the force must be applied.
    /// - `center_of_mass`: the center-of-mass (world-space) of the [`RigidBody`] the force is being
    ///   applied to.
    pub fn at_point(force: Vect, point: Vect, center_of_mass: Vect) -> Self {
        Self {
            force,
            #[cfg(feature = "dim2")]
            torque: (point - center_of_mass).perp_dot(force),
            #[cfg(feature = "dim3")]
            torque: (point - center_of_mass).cross(force),
        }
    }
}

impl Add for ExternalForce {
    type Output = Self;

    #[inline]
    fn add(mut self, rhs: Self) -> Self::Output {
        self += rhs;
        self
    }
}

impl Sub for ExternalForce {
    type Output = Self;

    #[inline]
    fn sub(mut self, rhs: Self) -> Self::Output {
        self -= rhs;
        self
    }
}

impl AddAssign for ExternalForce {
    #[inline]
    fn add_assign(&mut self, rhs: Self) {
        self.force += rhs.force;
        self.torque += rhs.torque;
    }
}

impl SubAssign for ExternalForce {
    #[inline]
    fn sub_assign(&mut self, rhs: Self) {
        self.force -= rhs.force;
        self.torque -= rhs.torque;
    }
}

/// Instantaneous external impulse applied continuously to a [`RigidBody`].
///
/// The impulse is only applied once, and whenever it modified (based
/// on Bevy’s change detection).
#[derive(Copy, Clone, Debug, Default, PartialEq, Component, Reflect)]
#[reflect(Component, PartialEq)]
pub struct ExternalImpulse {
    /// The linear impulse applied to the [`RigidBody`].
    pub impulse: Vect,
    /// The angular impulse applied to the [`RigidBody`].
    #[cfg(feature = "dim2")]
    pub torque_impulse: f32,
    /// The angular impulse applied to the [`RigidBody`].
    #[cfg(feature = "dim3")]
    pub torque_impulse: Vect,
}

impl ExternalImpulse {
    /// An impulse applied at a specific world-space point of a [`RigidBody`].
    ///
    /// # Parameters
    /// - `impulse`: the impulse to apply.
    /// - `point`: the point (world-space) where the impulse must be applied.
    /// - `center_of_mass`: the center-of-mass (world-space) of the [`RigidBody`] the impulse is being
    ///   applied to.
    pub fn at_point(impulse: Vect, point: Vect, center_of_mass: Vect) -> Self {
        Self {
            impulse,
            #[cfg(feature = "dim2")]
            torque_impulse: (point - center_of_mass).perp_dot(impulse),
            #[cfg(feature = "dim3")]
            torque_impulse: (point - center_of_mass).cross(impulse),
        }
    }

    /// Reset the external impulses to zero.
    pub fn reset(&mut self) {
        *self = Default::default();
    }
}

impl Add for ExternalImpulse {
    type Output = Self;

    #[inline]
    fn add(mut self, rhs: Self) -> Self::Output {
        self += rhs;
        self
    }
}

impl Sub for ExternalImpulse {
    type Output = Self;

    #[inline]
    fn sub(mut self, rhs: Self) -> Self::Output {
        self -= rhs;
        self
    }
}

impl AddAssign for ExternalImpulse {
    #[inline]
    fn add_assign(&mut self, rhs: Self) {
        self.impulse += rhs.impulse;
        self.torque_impulse += rhs.torque_impulse;
    }
}

impl SubAssign for ExternalImpulse {
    #[inline]
    fn sub_assign(&mut self, rhs: Self) {
        self.impulse -= rhs.impulse;
        self.torque_impulse -= rhs.torque_impulse;
    }
}

/// Gravity is multiplied by this scaling factor before it's
/// applied to this [`RigidBody`].
#[derive(Copy, Clone, Debug, PartialEq, Component, Reflect)]
#[reflect(Component, PartialEq)]
pub struct GravityScale(pub f32);

impl Default for GravityScale {
    fn default() -> Self {
        Self(1.0)
    }
}

/// If the `TimestepMode::Interpolated` mode is set and this component is present,
/// the associated [`RigidBody`] will have its position automatically interpolated
/// between the last two [`RigidBody`] positions set by the physics engine.
#[derive(Copy, Clone, Debug, Default, PartialEq, Component)]
pub struct TransformInterpolation {
    /// The starting point of the interpolation.
    pub start: Option<Isometry>,
    /// The end point of the interpolation.
    pub end: Option<Isometry>,
}

impl TransformInterpolation {
    /// Interpolates between the start and end positions with `t` in the range `[0..1]`.
    pub fn lerp_slerp(&self, t: f32) -> Option<Isometry> {
        if let (Some(start), Some(end)) = (self.start, self.end) {
            Some(start.lerp_slerp(end, t))
        } else {
            None
        }
    }
}

/// Indicates whether the [`RigidBody`] is disabled explicitly by the user.
#[derive(Copy, Clone, Default, Debug, PartialEq, Eq, Component, Reflect)]
#[reflect(Component, PartialEq)]
pub struct RigidBodyDisabled;

/// Set the additional number of solver iterations run for a rigid-body and
/// everything interacting with it.
///
/// Increasing this number will help improve simulation accuracy on this rigid-body
/// and every rigid-body interacting directly or indirectly with it (through joints
/// or contacts). This implies a performance hit.
///
/// The default value is 0, meaning exactly [`IntegrationParameters::num_solver_iterations`] will
/// be used as number of solver iterations for this body.
#[derive(Copy, Clone, Default, Debug, PartialEq, Eq, Component, Reflect)]
#[reflect(Component, PartialEq)]
pub struct AdditionalSolverIterations(pub usize);

/// Indicates whether the rigid-body was created in the physics backend.
#[derive(Copy, Clone, Default, Debug, PartialEq, Eq, Component, Reflect)]
#[reflect(Component, PartialEq)]
pub struct RigidBodyCreated;
