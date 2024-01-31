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

/// Steps the Bevy app until FixedUpdate is reached.
/// # Remarks
/// This function can execute any number of Updates and FixedUpdates.
#[cfg(test)]
pub(crate) fn step_fixed_update(app: &mut bevy::app::App) {
    // This can not be set to small values like one nanosecond, because it will cause nearly infinite loop.
    const FIXED_DELTA_TIME: f64 = 0.01;
    const MAX_WAIT_STEPS: usize = 32;

    #[derive(bevy::ecs::system::Resource)]
    struct FixedUpdateReacherResource {
        reached: bool,
    }

    fn fixed_update_reacher(mut resource: bevy::ecs::system::ResMut<FixedUpdateReacherResource>) {
        resource.reached = true;
    }

    app.add_systems(bevy::app::FixedUpdate, fixed_update_reacher)
        .insert_resource(FixedUpdateReacherResource { reached: false })
        .insert_resource(bevy::time::Time::<bevy::time::Fixed>::from_seconds(
            FIXED_DELTA_TIME,
        ));

    for _ in 0..MAX_WAIT_STEPS {
        app.update();

        if app
            .world
            .get_resource::<FixedUpdateReacherResource>()
            .unwrap()
            .reached
        {
            return;
        }

        std::thread::sleep(std::time::Duration::from_secs_f64(FIXED_DELTA_TIME))
    }

    // Do not wait for test break due to timeout if FixedUpdate is not reachable.
    panic!("FixedUpdate did not reach in {MAX_WAIT_STEPS} steps.");
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
