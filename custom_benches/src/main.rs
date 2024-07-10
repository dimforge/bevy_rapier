//! Translated from rapier benchmark.
//!
//! https://github.com/dimforge/rapier/blob/87ada34008f4a1a313ccf8c3040040bab4f10e69/benchmarks3d/many_pyramids3.rs

use benches_common::default_app;
use benches_common::wait_app_start;
use bevy::prelude::*;
use bevy_rapier3d::dynamics::RigidBody;
use bevy_rapier3d::geometry::Collider;
use bevy_rapier3d::math::Vect;
use bevy_rapier3d::plugin::RapierContext;

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

pub fn custom_bencher(steps: usize, setup: impl Fn(&mut App)) {
    let mut app = default_app();
    setup(&mut app);
    wait_app_start(&mut app);

    let mut timer_total = rapier3d::counters::Timer::new();
    let mut timer_full_update = rapier3d::counters::Timer::new();
    let mut rapier_step_times = vec![];
    let mut total_update_times = vec![];
    timer_total.start();
    for _ in 0..steps {
        timer_full_update.start();
        app.update();
        timer_full_update.pause();
        let elapsed_time = timer_full_update.time() as f32;
        let rc = app.world().resource::<RapierContext>();
        rapier_step_times.push(rc.pipeline.counters.step_time.time() as f32);
        total_update_times.push(elapsed_time);
    }
    timer_total.pause();
    let average_total = total_update_times.iter().sum::<f32>() / total_update_times.len() as f32;
    println!("average total time: {}", average_total);
    let average_rapier_step =
        rapier_step_times.iter().sum::<f32>() / rapier_step_times.len() as f32;
    println!("average rapier step time: {}", average_rapier_step);
    let average_rapier_overhead = average_total - average_rapier_step;
    println!("average bevy overhead: {}", average_rapier_overhead);
    println!("total time: {}", timer_total.time());
}

fn pyramid_1_with_height_2() {
    custom_bencher(1000, |app| setup_cubes(app, 1, 2));
}

fn pyramid_2_with_height_20() {
    custom_bencher(100, |app| setup_cubes(app, 3, 20));
}

fn main() {
    pyramid_1_with_height_2();
    pyramid_2_with_height_20();
}
