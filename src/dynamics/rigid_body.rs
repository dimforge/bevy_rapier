use crate::math::{Rot, Vect};
use bevy::prelude::*;
use rapier::prelude::{
    Isometry, LockedAxes as RapierLockedAxes, RigidBodyActivation, RigidBodyHandle,
    RigidBodyMassProps, RigidBodyType,
};

#[derive(Copy, Clone, Debug, Component)]
pub struct RapierRigidBodyHandle(pub RigidBodyHandle);

#[derive(Copy, Clone, Debug, PartialEq, Eq, Component, Reflect)]
pub enum RigidBody {
    Dynamic,
    Fixed,
    KinematicPositionBased,
    KinematicVelocityBased,
}

impl Default for RigidBody {
    fn default() -> Self {
        RigidBody::Dynamic
    }
}

impl Into<RigidBodyType> for RigidBody {
    fn into(self) -> RigidBodyType {
        match self {
            RigidBody::Dynamic => RigidBodyType::Dynamic,
            RigidBody::Fixed => RigidBodyType::Fixed,
            RigidBody::KinematicPositionBased => RigidBodyType::KinematicPositionBased,
            RigidBody::KinematicVelocityBased => RigidBodyType::KinematicVelocityBased,
        }
    }
}

#[derive(Copy, Clone, Debug, Default, Component, Reflect)]
pub struct Velocity {
    pub linvel: Vect,
    #[cfg(feature = "dim2")]
    pub angvel: f32,
    #[cfg(feature = "dim3")]
    pub angvel: Vect,
}

impl Velocity {
    pub fn zero() -> Self {
        Self::default()
    }
}

#[derive(Copy, Clone, Debug, Default, Component, Reflect)]
pub struct AdditionalMassProperties(pub MassProperties);

#[derive(Copy, Clone, Debug, Default, Component, Reflect)]
pub struct MassProperties {
    pub local_center_of_mass: Vect,
    pub mass: f32,
    #[cfg(feature = "dim2")]
    pub principal_inertia: f32,
    #[cfg(feature = "dim3")]
    pub principal_inertia_local_frame: Rot,
    #[cfg(feature = "dim3")]
    pub principal_inertia: Vect,
}

impl MassProperties {
    #[cfg(feature = "dim2")]
    pub fn into_rapier(self, physics_scale: f32) -> rapier::dynamics::MassProperties {
        rapier::dynamics::MassProperties::new(
            (self.local_center_of_mass / physics_scale).into(),
            self.mass,
            (self.principal_inertia / (physics_scale * physics_scale)).into(),
        )
    }

    #[cfg(feature = "dim3")]
    pub fn into_rapier(self, physics_scale: f32) -> rapier::dynamics::MassProperties {
        rapier::dynamics::MassProperties::with_principal_inertia_frame(
            (self.local_center_of_mass / physics_scale).into(),
            self.mass,
            (self.principal_inertia / (physics_scale * physics_scale)).into(),
            self.principal_inertia_local_frame.into(),
        )
    }

    pub fn from_rapier(mprops: rapier::dynamics::MassProperties, physics_scale: f32) -> Self {
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

bitflags::bitflags! {
    #[derive(Default, Component, Reflect)]
    /// Flags affecting the behavior of the constraints solver for a given contact manifold.
    pub struct LockedAxes: u8 {
        /// Flag indicating that the rigid-body cannot translate along the `X` axis.
        const TRANSLATION_LOCKED_X = 1 << 0;
        /// Flag indicating that the rigid-body cannot translate along the `Y` axis.
        const TRANSLATION_LOCKED_Y = 1 << 1;
        /// Flag indicating that the rigid-body cannot translate along the `Z` axis.
        const TRANSLATION_LOCKED_Z = 1 << 2;
        /// Flag indicating that the rigid-body cannot translate along any direction.
        const TRANSLATION_LOCKED = Self::TRANSLATION_LOCKED_X.bits | Self::TRANSLATION_LOCKED_Y.bits | Self::TRANSLATION_LOCKED_Z.bits;
        /// Flag indicating that the rigid-body cannot rotate along the `X` axis.
        const ROTATION_LOCKED_X = 1 << 3;
        /// Flag indicating that the rigid-body cannot rotate along the `Y` axis.
        const ROTATION_LOCKED_Y = 1 << 4;
        /// Flag indicating that the rigid-body cannot rotate along the `Z` axis.
        const ROTATION_LOCKED_Z = 1 << 5;
        /// Combination of flags indicating that the rigid-body cannot rotate along any axis.
        const ROTATION_LOCKED = Self::ROTATION_LOCKED_X.bits | Self::ROTATION_LOCKED_Y.bits | Self::ROTATION_LOCKED_Z.bits;
    }
}

impl Into<RapierLockedAxes> for LockedAxes {
    fn into(self) -> RapierLockedAxes {
        RapierLockedAxes::from_bits(self.bits()).expect("Internal conversion error.")
    }
}

#[derive(Copy, Clone, Debug, Default, Component, Reflect)]
pub struct ExternalForce {
    pub force: Vect,
    #[cfg(feature = "dim2")]
    pub torque: f32,
    #[cfg(feature = "dim3")]
    pub torque: Vect,
}

#[derive(Copy, Clone, Debug, Default, Component, Reflect)]
pub struct ExternalImpulse {
    pub impulse: Vect,
    #[cfg(feature = "dim2")]
    pub torque_impulse: f32,
    #[cfg(feature = "dim3")]
    pub torque_impulse: Vect,
}

#[derive(Copy, Clone, Debug, Component, Reflect)]
pub struct GravityScale(pub f32);

impl Default for GravityScale {
    fn default() -> Self {
        Self(1.0)
    }
}

#[derive(Copy, Clone, Debug, Default, Component, Reflect)]
pub struct Ccd {
    pub enabled: bool,
}

#[derive(Copy, Clone, Debug, Default, Component, Reflect)]
pub struct Dominance {
    pub groups: i8,
}

#[derive(Copy, Clone, Debug, Component, Reflect)]
pub struct Sleeping {
    pub linear_threshold: f32,
    pub angular_threshold: f32,
    pub sleeping: bool,
}

impl Sleeping {
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

#[derive(Copy, Clone, Debug, Component, Reflect)]
pub struct Damping {
    pub linear_damping: f32,
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

#[derive(Copy, Clone, Debug, Component)]
pub struct TransformInterpolation {
    pub start: Option<Isometry<f32>>,
    pub end: Option<Isometry<f32>>,
}

impl Default for TransformInterpolation {
    fn default() -> Self {
        Self {
            start: None,
            end: None,
        }
    }
}

impl TransformInterpolation {
    pub fn lerp_slerp(&self, t: f32) -> Option<Isometry<f32>> {
        if let (Some(start), Some(end)) = (self.start, self.end) {
            Some(start.lerp_slerp(&end, t))
        } else {
            None
        }
    }
}
