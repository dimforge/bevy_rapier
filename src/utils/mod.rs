use bevy::prelude::Transform;
use rapier::math::{Isometry, Real};

/// Converts a Rapier isometry to a Bevy transform.
///
/// The translation is multiplied by the `physics_scale`.
#[cfg(feature = "dim2")]
pub fn iso_to_transform(iso: &Isometry<Real>, physics_scale: Real) -> Transform {
    Transform {
        translation: (iso.translation.vector.push(0.0) * physics_scale).into(),
        rotation: bevy::prelude::Quat::from_rotation_z(iso.rotation.angle()),
        ..Default::default()
    }
}

/// Converts a Rapier isometry to a Bevy transform.
///
/// The translation is multiplied by the `physics_scale`.
#[cfg(feature = "dim3")]
pub fn iso_to_transform(iso: &Isometry<Real>, physics_scale: Real) -> Transform {
    Transform {
        translation: (iso.translation.vector * physics_scale).into(),
        rotation: iso.rotation.into(),
        ..Default::default()
    }
}

/// Converts a Bevy transform to a Rapier isometry.
///
/// The translation is divided by the `physics_scale`.
#[cfg(feature = "dim2")]
pub(crate) fn transform_to_iso(transform: &Transform, physics_scale: Real) -> Isometry<Real> {
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
pub(crate) fn transform_to_iso(transform: &Transform, physics_scale: Real) -> Isometry<Real> {
    Isometry::from_parts(
        (transform.translation / physics_scale).into(),
        transform.rotation.into(),
    )
}

#[cfg(test)]
#[cfg(feature = "dim3")]
mod tests {
    use bevy::prelude::Transform;
    use super::*;

    #[test]
    fn convert_back_to_equal_transform() {
        let transform = Transform {
            translation: bevy::prelude::Vec3::new(-2.1855694e-8, 0.0, 0.0),
            rotation: bevy::prelude::Quat::from_xyzw(0.99999994, 0.0, 1.6292068e-7, 0.0).normalize(),
            ..Default::default()
        };
        let converted_transform = iso_to_transform(&transform_to_iso(&transform, 1.0), 1.0);
        assert_eq!(converted_transform, transform);
    }
}
