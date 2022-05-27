use bevy::prelude::GlobalTransform;
use rapier::math::{Isometry, Real};

/// Converts a Rapier isometry to a Bevy transform.
///
/// The translation is multiplied by the `physics_scale`.
#[cfg(feature = "dim2")]
pub fn iso_to_transform(iso: &Isometry<Real>, physics_scale: Real) -> GlobalTransform {
    GlobalTransform {
        translation: (iso.translation.vector.push(0.0) * physics_scale).into(),
        rotation: bevy::prelude::Quat::from_axis_angle(
            bevy::prelude::Vec3::Z,
            iso.rotation.angle(),
        ),
        ..Default::default()
    }
}

/// Converts a Rapier isometry to a Bevy transform.
///
/// The translation is multiplied by the `physics_scale`.
#[cfg(feature = "dim3")]
pub fn iso_to_transform(iso: &Isometry<Real>, physics_scale: Real) -> GlobalTransform {
    GlobalTransform {
        translation: (iso.translation.vector * physics_scale).into(),
        rotation: iso.rotation.into(),
        ..Default::default()
    }
}

/// Converts a Bevy transform to a Rapier isometry.
///
/// The translation is divided by the `physics_scale`.
#[cfg(feature = "dim2")]
pub(crate) fn transform_to_iso(transform: &GlobalTransform, physics_scale: Real) -> Isometry<Real> {
    use bevy::math::Vec3Swizzles;
    Isometry::new(
        (transform.translation / physics_scale).xy().into(),
        transform.rotation.to_scaled_axis().z,
    )
}

/// Converts a Bevy transform to a Rapier isometry.
///
/// The translation is divided by the `physics_scale`.
#[cfg(feature = "dim3")]
pub(crate) fn transform_to_iso(transform: &GlobalTransform, physics_scale: Real) -> Isometry<Real> {
    Isometry::new(
        (transform.translation / physics_scale).into(),
        transform.rotation.to_scaled_axis().into(),
    )
}
