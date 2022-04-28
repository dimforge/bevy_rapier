pub use self::configuration::RapierConfiguration;
pub use self::context::RapierContext;
pub use self::plugin::{NoUserData, RapierPhysicsPlugin};
pub use self::systems::{ColliderComponents, RigidBodyComponents};

mod configuration;
mod context;
mod narrow_phase;
mod plugin;
pub(crate) mod systems;

use bevy::prelude::Transform;
use rapier::math::{Isometry, Real};

#[cfg(feature = "dim2")]
pub(crate) fn iso_to_transform(iso: &Isometry<Real>, physics_scale: Real) -> Transform {
    Transform {
        translation: (iso.translation.vector.push(0.0) * physics_scale).into(),
        rotation: bevy::prelude::Quat::from_axis_angle(
            bevy::prelude::Vec3::Z,
            iso.rotation.angle(),
        ),
        ..Default::default()
    }
}

#[cfg(feature = "dim3")]
pub(crate) fn iso_to_transform(iso: &Isometry<Real>, physics_scale: Real) -> Transform {
    Transform {
        translation: (iso.translation.vector * physics_scale).into(),
        rotation: iso.rotation.into(),
        ..Default::default()
    }
}

#[cfg(feature = "dim2")]
pub(crate) fn transform_to_iso(transform: &Transform, physics_scale: Real) -> Isometry<Real> {
    use bevy::math::Vec3Swizzles;
    Isometry::new(
        (transform.translation / physics_scale).xy().into(),
        transform.rotation.to_scaled_axis().z,
    )
}

#[cfg(feature = "dim3")]
pub(crate) fn transform_to_iso(transform: &Transform, physics_scale: Real) -> Isometry<Real> {
    Isometry::new(
        (transform.translation / physics_scale).into(),
        transform.rotation.to_scaled_axis().into(),
    )
}
