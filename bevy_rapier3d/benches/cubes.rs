//! Translated from avian benchmark.

use benches_common::bench_app;
use bevy::prelude::*;
use bevy_rapier3d::math::*;
use bevy_rapier3d::prelude::*;

fn setup_cubes(app: &mut App, size: u32) {
    app.add_systems(Startup, move |mut commands: Commands| {
        commands.spawn((
            RigidBody::Fixed,
            Transform::from_translation(-2.0 * Vect::Z),
            Collider::cuboid(100.0, 1.0, 100.0),
        ));
        for x in 0..size {
            for z in 0..size {
                commands.spawn((
                    RigidBody::Dynamic,
                    Transform::from_translation(Vec3::new(x as f32, 2.0, z as f32)),
                    Collider::cuboid(1.0, 1.0, 1.0),
                ));
            }
        }
    });
}

#[divan::bench]
fn cubes_3x3_30_steps(bencher: divan::Bencher) {
    bench_app(bencher, 30, |app| setup_cubes(app, 3))
}
#[divan::bench]
fn cubes_5x5_30_steps(bencher: divan::Bencher) {
    bench_app(bencher, 30, |app| setup_cubes(app, 5))
}
#[divan::bench]
fn cubes_10x10_30_steps(bencher: divan::Bencher) {
    bench_app(bencher, 30, |app| setup_cubes(app, 10))
}

fn main() {
    // Run registered benchmarks.
    divan::main();
}
