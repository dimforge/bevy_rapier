use crate::math::Vect;
use bevy::prelude::*;
use rapier::prelude::{
    Isometry, LockedAxes as RapierLockedAxes, RigidBodyActivation, RigidBodyHandle, RigidBodyType,
};
use std::ops::{Add, AddAssign, Sub, SubAssign};

/// The Rapier handle of a [`RigidBody`] that was inserted to the physics scene.
#[derive(Copy, Clone, Debug, Component)]
pub struct RapierRigidBodyHandle(pub RigidBodyHandle);

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
    /// modified by the user and is independent from any contact or joint it is involved in.
    KinematicPositionBased,
    /// A `RigidBody::KinematicVelocityBased` body cannot be affected by any external forces but can be controlled
    /// by the user at the velocity level while keeping realistic one-way interaction with dynamic bodies.
    ///
    /// One-way interaction means that a kinematic body can push a dynamic body, but a kinematic body
    /// cannot be pushed by anything. In other words, the trajectory of a kinematic body can only be
    /// modified by the user and is independent from any contact or joint it is involved in.
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

/// The velocity of a [`RigidBody`].
///
/// Use this component to control and/or read the velocity of a dynamic or kinematic [`RigidBody`].
/// If this component isn’t present, a dynamic [`RigidBody`] will still be able to move (you will just
/// not be able to read/modify its velocity).
///
/// This only affects entities with a [`RigidBody`] component.
#[derive(Copy, Clone, Debug, Default, PartialEq, Component, Reflect)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[reflect(Component, PartialEq)]
pub struct Velocity {
    /// The linear velocity of the [`RigidBody`].
    pub linvel: Vect,
    /// The angular velocity of the [`RigidBody`] in radian per second.
    #[cfg(feature = "dim2")]
    pub angvel: f32,
    /// The angular velocity of the [`RigidBody`].
    #[cfg(feature = "dim3")]
    pub angvel: Vect,
}

impl Velocity {
    /// Initialize a velocity set to zero.
    pub const fn zero() -> Self {
        Self {
            linvel: Vect::ZERO,
            #[cfg(feature = "dim2")]
            angvel: 0.0,
            #[cfg(feature = "dim3")]
            angvel: Vect::ZERO,
        }
    }

    /// Initialize a velocity with the given linear velocity, and an angular velocity of zero.
    pub const fn linear(linvel: Vect) -> Self {
        Self {
            linvel,
            #[cfg(feature = "dim2")]
            angvel: 0.0,
            #[cfg(feature = "dim3")]
            angvel: Vect::ZERO,
        }
    }

    /// Initialize a velocity with the given angular velocity, and a linear velocity of zero.
    #[cfg(feature = "dim2")]
    pub const fn angular(angvel: f32) -> Self {
        Self {
            linvel: Vect::ZERO,
            angvel,
        }
    }

    /// Initialize a velocity with the given angular velocity, and a linear velocity of zero.
    #[cfg(feature = "dim3")]
    pub const fn angular(angvel: Vect) -> Self {
        Self {
            linvel: Vect::ZERO,
            angvel,
        }
    }

    /// Get linear velocity of specific world-space point of a [`RigidBody`].
    ///
    /// # Parameters
    /// - `point`: the point (world-space) to compute the velocity for.
    /// - `center_of_mass`: the center-of-mass (world-space) of the [`RigidBody`] the velocity belongs to.
    pub fn linear_velocity_at_point(&self, point: Vect, center_of_mass: Vect) -> Vect {
        #[cfg(feature = "dim2")]
        return self.linvel + self.angvel * (point - center_of_mass).perp();

        #[cfg(feature = "dim3")]
        return self.linvel + self.angvel.cross(point - center_of_mass);
    }
}

/// Mass-properties of a [`RigidBody`], added to the contributions of its attached colliders.
///
/// This only affects entities with a [`RigidBody`] component.
#[derive(Copy, Clone, Debug, PartialEq, Component, Reflect)]
#[reflect(Component, PartialEq)]
pub enum AdditionalMassProperties {
    /// This mass will be added to the [`RigidBody`]. The rigid-body’s total
    /// angular inertia tensor (obtained from its attached colliders) will
    /// be scaled accordingly.
    Mass(f32),
    /// These mass properties will be added to the [`RigidBody`].
    MassProperties(MassProperties),
}

impl Default for AdditionalMassProperties {
    fn default() -> Self {
        Self::MassProperties(MassProperties::default())
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

/// Center-of-mass, mass, and angular inertia.
///
/// This cannot be used as a component. Use the components `ReadMassProperties` to read a [`RigidBody`]’s
/// mass-properties or `AdditionalMassProperties` to set its additional mass-properties.
#[derive(Copy, Clone, Debug, Default, PartialEq, Reflect)]
#[reflect(PartialEq)]
pub struct MassProperties {
    /// The center of mass of a [`RigidBody`] expressed in its local-space.
    pub local_center_of_mass: Vect,
    /// The mass of a [`RigidBody`].
    pub mass: f32,
    /// The principal angular inertia of the [`RigidBody`].
    #[cfg(feature = "dim2")]
    pub principal_inertia: f32,
    /// The principal vectors of the local angular inertia tensor of the [`RigidBody`].
    #[cfg(feature = "dim3")]
    pub principal_inertia_local_frame: crate::math::Rot,
    /// The principal angular inertia of the [`RigidBody`].
    #[cfg(feature = "dim3")]
    pub principal_inertia: Vect,
}

impl MassProperties {
    /// Converts these mass-properties to Rapier’s `MassProperties` structure.
    #[cfg(feature = "dim2")]
    pub fn into_rapier(self, physics_scale: f32) -> rapier::dynamics::MassProperties {
        rapier::dynamics::MassProperties::new(
            (self.local_center_of_mass / physics_scale).into(),
            self.mass,
            #[allow(clippy::useless_conversion)] // Need to convert if dim3 enabled
            (self.principal_inertia / (physics_scale * physics_scale)).into(),
        )
    }

    /// Converts these mass-properties to Rapier’s `MassProperties` structure.
    #[cfg(feature = "dim3")]
    pub fn into_rapier(self, physics_scale: f32) -> rapier::dynamics::MassProperties {
        rapier::dynamics::MassProperties::with_principal_inertia_frame(
            (self.local_center_of_mass / physics_scale).into(),
            self.mass,
            (self.principal_inertia / (physics_scale * physics_scale)).into(),
            self.principal_inertia_local_frame.into(),
        )
    }

    /// Converts Rapier’s `MassProperties` structure to `Self`.
    pub fn from_rapier(mprops: rapier::dynamics::MassProperties, physics_scale: f32) -> Self {
        #[allow(clippy::useless_conversion)] // Need to convert if dim3 enabled
        Self {
            mass: mprops.mass(),
            local_center_of_mass: (mprops.local_com * physics_scale).into(),
            principal_inertia: (mprops.principal_inertia() * (physics_scale * physics_scale))
                .into(),
            #[cfg(feature = "dim3")]
            principal_inertia_local_frame: mprops.principal_inertia_local_frame.into(),
        }
    }
}

#[derive(Default, Component, Reflect, Copy, Clone, Ord, PartialOrd, Eq, PartialEq, Hash)]
#[reflect(Component, PartialEq)]
/// Flags affecting the behavior of the constraints solver for a given contact manifold.
pub struct LockedAxes(u8);

bitflags::bitflags! {
    impl LockedAxes: u8 {
        /// Flag indicating that the [`RigidBody`] cannot translate along the `X` axis.
        const TRANSLATION_LOCKED_X = 1 << 0;
        /// Flag indicating that the [`RigidBody`] cannot translate along the `Y` axis.
        const TRANSLATION_LOCKED_Y = 1 << 1;
        /// Flag indicating that the [`RigidBody`] cannot translate along the `Z` axis.
        const TRANSLATION_LOCKED_Z = 1 << 2;
        /// Flag indicating that the [`RigidBody`] cannot translate along any direction.
        const TRANSLATION_LOCKED = Self::TRANSLATION_LOCKED_X.bits() | Self::TRANSLATION_LOCKED_Y.bits() | Self::TRANSLATION_LOCKED_Z.bits();
        /// Flag indicating that the [`RigidBody`] cannot rotate along the `X` axis.
        const ROTATION_LOCKED_X = 1 << 3;
        /// Flag indicating that the [`RigidBody`] cannot rotate along the `Y` axis.
        const ROTATION_LOCKED_Y = 1 << 4;
        /// Flag indicating that the [`RigidBody`] cannot rotate along the `Z` axis.
        const ROTATION_LOCKED_Z = 1 << 5;
        /// Combination of flags indicating that the [`RigidBody`] cannot rotate along any axis.
        const ROTATION_LOCKED = Self::ROTATION_LOCKED_X.bits() | Self::ROTATION_LOCKED_Y.bits() | Self::ROTATION_LOCKED_Z.bits();
    }
}

impl From<LockedAxes> for RapierLockedAxes {
    fn from(locked_axes: LockedAxes) -> RapierLockedAxes {
        RapierLockedAxes::from_bits(locked_axes.bits()).expect("Internal conversion error.")
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
    /// - `point`: the point (world-space) where the impulse must be applied.
    /// - `center_of_mass`: the center-of-mass (world-space) of the [`RigidBody`] the impulse is being
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
/// The impulse is only applied once, and whenever it it modified (based
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

/// Information used for Continuous-Collision-Detection.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Component, Reflect)]
#[reflect(Component, PartialEq)]
pub struct Ccd {
    /// Is CCD enabled for this [`RigidBody`]?
    pub enabled: bool,
}

impl Ccd {
    /// Enable CCD for a [`RigidBody`].
    pub fn enabled() -> Self {
        Self { enabled: true }
    }

    /// Disable CCD for a [`RigidBody`].
    ///
    /// Note that a [`RigidBody`] without the Ccd component attached
    /// has CCD disabled by default.
    pub fn disabled() -> Self {
        Self { enabled: false }
    }
}

/// The dominance groups of a [`RigidBody`].
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Component, Reflect)]
#[reflect(Component, PartialEq)]
pub struct Dominance {
    // FIXME: rename this to `group` (no `s`).
    /// The dominance groups of a [`RigidBody`].
    pub groups: i8,
}

impl Dominance {
    /// Initialize the dominance to the given group.
    pub fn group(group: i8) -> Self {
        Self { groups: group }
    }
}

/// The activation status of a body.
///
/// This controls whether a body is sleeping or not.
/// If the threshold is negative, the body never sleeps.
#[derive(Copy, Clone, Debug, PartialEq, Component, Reflect)]
#[reflect(Component, PartialEq)]
pub struct Sleeping {
    /// The linear velocity below which the body can fall asleep.
    pub linear_threshold: f32,
    /// The angular velocity below which the body can fall asleep.
    pub angular_threshold: f32,
    /// Is this body sleeping?
    pub sleeping: bool,
}

impl Sleeping {
    /// Creates a components that disables sleeping for the associated [`RigidBody`].
    pub fn disabled() -> Self {
        Self {
            linear_threshold: -1.0,
            angular_threshold: -1.0,
            sleeping: false,
        }
    }
}

impl Default for Sleeping {
    fn default() -> Self {
        Self {
            linear_threshold: RigidBodyActivation::default_linear_threshold(),
            angular_threshold: RigidBodyActivation::default_angular_threshold(),
            sleeping: false,
        }
    }
}

/// Damping factors to gradually slow down a [`RigidBody`].
#[derive(Copy, Clone, Debug, PartialEq, Component, Reflect)]
#[reflect(Component, PartialEq)]
pub struct Damping {
    // TODO: rename these to "linear" and "angular"?
    /// Damping factor for gradually slowing down the translational motion of the [`RigidBody`].
    pub linear_damping: f32,
    /// Damping factor for gradually slowing down the angular motion of the [`RigidBody`].
    pub angular_damping: f32,
}

impl Default for Damping {
    fn default() -> Self {
        Self {
            linear_damping: 0.0,
            angular_damping: 0.0,
        }
    }
}

/// If the `TimestepMode::Interpolated` mode is set and this component is present,
/// the associated [`RigidBody`] will have its position automatically interpolated
/// between the last two [`RigidBody`] positions set by the physics engine.
#[derive(Copy, Clone, Debug, Default, PartialEq, Component)]
pub struct TransformInterpolation {
    /// The starting point of the interpolation.
    pub start: Option<Isometry<f32>>,
    /// The end point of the interpolation.
    pub end: Option<Isometry<f32>>,
}

impl TransformInterpolation {
    /// Interpolates between the start and end positions with `t` in the range `[0..1]`.
    pub fn lerp_slerp(&self, t: f32) -> Option<Isometry<f32>> {
        if let (Some(start), Some(end)) = (self.start, self.end) {
            Some(start.lerp_slerp(&end, t))
        } else {
            None
        }
    }
}

/// Indicates whether or not the [`RigidBody`] is disabled explicitly by the user.
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
