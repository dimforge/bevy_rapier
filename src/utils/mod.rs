use bevy::prelude::Transform;
use rapier::math::{Isometry, Real};

pub mod as_real;
pub use as_real::*;

/// Converts a Rapier isometry to a Bevy transform.
///
/// The translation is multiplied by the `physics_scale`.
#[cfg(feature = "dim2")]
pub fn iso_to_transform(iso: &Isometry<Real>, physics_scale: Real) -> Transform {
    #[cfg(feature = "f32")]
    let (translation, rotation) = {
        use bevy::math::{Vec3, Quat};
        let translation: Vec3 = iso.translation.vector.into();
        let rotation = Quat::from_rotation_z(iso.rotation.angle());
        (translation, rotation)
    };
    #[cfg(feature = "f64")]
    let (translation, rotation) = {
        use bevy::math::{DVec3, DQuat};
        let translation: DVec3 = iso.translation.vector.into();
        let rotation = DQuat::from_rotation_z(iso.rotation.angle());
        (translation, rotation)
    };
    Transform {
        translation: translation.as_single(),
        rotation: rotation.as_single(),
        ..Default::default()
    }
}

/// Converts a Rapier isometry to a Bevy transform.
///
/// The translation is multiplied by the `physics_scale`.
#[cfg(feature = "dim3")]
pub fn iso_to_transform(iso: &Isometry<Real>, physics_scale: Real) -> Transform {
    #[cfg(feature = "f32")]
    let (translation, rotation) = {
        use bevy::math::{Vec3, Quat};
        let translation: Vec3 = iso.translation.vector.into();
        let rotation: Quat = iso.rotation.into();
        (translation, rotation)
    };
    #[cfg(feature = "f64")]
    let (translation, rotation) = {
        use bevy::math::{DVec3, DQuat};
        let translation: DVec3 = iso.translation.vector.into();
        let rotation: DQuat = iso.rotation.into();
        (translation, rotation)
    };
    Transform {
        translation: (translation * physics_scale).as_single(),
        rotation: rotation.as_single(),
        ..Default::default()
    }
}

/// Converts a Bevy transform to a Rapier isometry.
///
/// The translation is divided by the `physics_scale`.
#[cfg(feature = "dim2")]
pub(crate) fn transform_to_iso(transform: &Transform, physics_scale: Real) -> Isometry<Real> {
    use bevy::math::Vec3Swizzles;
    let translation = transform.translation.as_real();
    let rotation = transform.rotation.to_scaled_axis().z.as_real();
    Isometry::new(
        (translation / physics_scale).xy().into(),
        rotation,
    )
}

/// Converts a Bevy transform to a Rapier isometry.
///
/// The translation is divided by the `physics_scale`.
#[cfg(feature = "dim3")]
pub(crate) fn transform_to_iso(transform: &Transform, physics_scale: Real) -> Isometry<Real> {
    let translation = transform.translation.as_real();
    let rotation = transform.rotation.as_real();
    Isometry::from_parts(
        (translation / physics_scale).into(),
        rotation.into(),
    )
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
