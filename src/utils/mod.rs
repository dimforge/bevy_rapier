use crate::math::*;
use bevy::prelude::Transform;
use rapier::math::Isometry;

/// Conversions to various precisions for interop reasons.
pub mod as_precise;

/// Converts a Rapier isometry to a Bevy transform.
///
/// The translation is multiplied by the `physics_scale`.
#[cfg(feature = "dim2")]
pub fn iso_to_transform(iso: &Isometry<Real>, physics_scale: Real) -> Transform {
    use bevy::math::Quat;
    let translation = Vect::from(iso.translation.vector) * physics_scale;
    let rotation = Quat::from_rotation_z(iso.rotation.angle().as_single());
    Transform {
        translation: translation.as_single().extend(0.0),
        rotation,
        ..Default::default()
    }
}

/// Converts a Rapier isometry to a Bevy transform.
///
/// The translation is multiplied by the `physics_scale`.
#[cfg(feature = "dim3")]
pub fn iso_to_transform(iso: &Isometry<Real>, physics_scale: Real) -> Transform {
    let translation = (Vect::from(iso.translation.vector) * physics_scale).as_single();
    let rotation = Rot::from(iso.rotation).as_single();
    Transform {
        translation,
        rotation,
        ..Default::default()
    }
}

/// Converts a Bevy transform to a Rapier isometry.
///
/// The translation is divided by the `physics_scale`.
#[cfg(feature = "dim2")]
pub(crate) fn transform_to_iso(transform: &Transform, physics_scale: Real) -> Isometry<Real> {
    use bevy::math::{EulerRot, Vec3Swizzles};
    let translation = transform.translation.as_precise() / physics_scale;
    let rotation = transform.rotation.to_euler(EulerRot::ZYX).0.as_precise();
    Isometry::new(translation.xy().into(), rotation)
}

/// Converts a Bevy transform to a Rapier isometry.
///
/// The translation is divided by the `physics_scale`.
#[cfg(feature = "dim3")]
pub(crate) fn transform_to_iso(transform: &Transform, physics_scale: Real) -> Isometry<Real> {
    let translation = transform.translation.as_precise() / physics_scale;
    let rotation = transform.rotation.as_precise();
    Isometry::from_parts(translation.into(), rotation.into())
}

#[cfg(test)]
#[cfg(feature = "dim3")]
mod tests {
    use super::*;
    use bevy::prelude::Transform;

    #[test]
    fn convert_back_to_equal_transform() {
        let transform = Transform {
            translation: bevy::prelude::Vec3::new(-2.1855694e-8, 0.0, 0.0),
            rotation: bevy::prelude::Quat::from_xyzw(0.99999994, 0.0, 1.6292068e-7, 0.0)
                .normalize(),
            ..Default::default()
        };
        let converted_transform = iso_to_transform(&transform_to_iso(&transform, 1.0), 1.0);
        assert_eq!(converted_transform, transform);
    }
}
