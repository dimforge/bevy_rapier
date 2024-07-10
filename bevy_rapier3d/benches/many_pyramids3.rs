//! Translated from rapier benchmark.
//!
//! <https://github.com/dimforge/rapier/blob/87ada34008f4a1a313ccf8c3040040bab4f10e69/benchmarks3d/many_pyramids3.rs>

use benches_common::bench_app_updates;
use bevy::prelude::*;
use bevy_rapier3d::math::*;
use bevy_rapier3d::prelude::*;

pub fn create_pyramid(commands: &mut Commands, offset: Vect, stack_height: usize, rad: f32) {
    let shift = rad * 2.0;

    for i in 0usize..stack_height {
        for j in i..stack_height {
            let fj = j as f32;
            let fi = i as f32;
            let x = (fi * shift / 2.0) + (fj - fi) * shift;
            let y = fi * shift;

            // Build the rigid body.
            commands.spawn((
                RigidBody::Dynamic,
                Transform::from_translation(Vec3::new(x, y, 0.0) + offset),
                Collider::cuboid(1.0, 1.0, 1.0),
            ));
        }
    }
}

pub fn setup_cubes(app: &mut App, pyramid_count: usize, stack_height: usize) {
    app.add_systems(Startup, move |mut commands: Commands| {
        let rad = 0.5;
        let spacing = 4.0;

        /*
         * Ground
         */
        let ground_size = 50.0;
        let ground_height = 0.1;

        commands.spawn((
            RigidBody::Fixed,
            Transform::from_translation(Vect::new(0.0, -ground_height, 0.0)),
            Collider::cuboid(
                ground_size,
                ground_height,
                pyramid_count as f32 * spacing / 2.0 + ground_size,
            ),
        ));

        /*
         * Create the cubes
         */
        for pyramid_index in 0..pyramid_count {
            let bottomy = rad;
            create_pyramid(
                &mut commands,
                Vect::new(
                    0.0,
                    bottomy,
                    (pyramid_index as f32 - pyramid_count as f32 / 2.0) * spacing,
                ),
                stack_height,
                rad,
            );
        }
    });
}

#[divan::bench(sample_count = 60, sample_size = 1)]
fn pyramid_1_with_height_2(bencher: divan::Bencher) {
    bench_app_updates(bencher, |app| setup_cubes(app, 1, 2));
}

#[divan::bench(sample_count = 60, sample_size = 1)]
fn pyramid_1_with_height_20(bencher: divan::Bencher) {
    bench_app_updates(bencher, |app| setup_cubes(app, 1, 20));
}

#[divan::bench(sample_count = 60, sample_size = 1)]
fn pyramid_2_with_height_20(bencher: divan::Bencher) {
    bench_app_updates(bencher, |app| setup_cubes(app, 2, 20));
}

fn main() {
    // Run registered benchmarks.
    divan::main();
}
