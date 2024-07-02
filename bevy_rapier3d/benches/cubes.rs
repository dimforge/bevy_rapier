use std::time::Duration;

use benches_common::bench_app;
use bevy::prelude::*;
use bevy_rapier3d::math::*;
use bevy_rapier3d::prelude::*;
use criterion::{criterion_group, criterion_main, Criterion};

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

fn criterion_benchmark(c: &mut Criterion) {
    c.bench_function("cubes 3x3, 30 steps", |b| {
        bench_app(b, 30, |app| setup_cubes(app, 3))
    });

    c.bench_function("cubes 5x5, 30 steps", |b| {
        bench_app(b, 30, |app| setup_cubes(app, 5))
    });

    c.bench_function("cubes 10x10, 30 steps", |b| {
        bench_app(b, 30, |app| setup_cubes(app, 10))
    });
}

criterion_group!(
    name = benches;
    config = Criterion::default().measurement_time(Duration::from_secs(20));
    targets = criterion_benchmark
);
criterion_main!(benches);
