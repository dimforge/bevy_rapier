//! Translated from rapier benchmark.

use benches_common::default_app;
use benches_common::wait_app_start;
use bevy::prelude::*;
use bevy_rapier3d::plugin::RapierContext;
use bevy_rapier3d::plugin::SimulationToRenderTime;
use many_pyramids3::setup_cubes;
mod many_pyramids3;

pub fn custom_bencher(steps: usize, setup: impl Fn(&mut App)) {
    let mut app = default_app();
    setup(&mut app);
    wait_app_start(&mut app);

    let mut timer_total = rapier3d::counters::Timer::new();
    let mut timer_full_update = rapier3d::counters::Timer::new();
    let mut bevy_overheads = vec![];
    let mut rapier_step_times = vec![];
    let mut total_update_times = vec![];
    timer_total.start();
    for _ in 0..steps {
        timer_full_update.start();
        app.update();
        timer_full_update.pause();
        let elapsed_time = timer_full_update.time() as f32;
        let rc = app.world().resource::<RapierContext>();
        let bevy_overhead = app.world().resource::<SimulationToRenderTime>();
        bevy_overheads.push(bevy_overhead.diff);
        rapier_step_times.push(rc.pipeline.counters.step_time.time() as f32);
        total_update_times.push(elapsed_time);
    }
    timer_total.pause();
    let average = bevy_overheads.iter().sum::<f32>() / bevy_overheads.len() as f32;
    println!("average bevy overhead: {}", average);
    let average = total_update_times.iter().sum::<f32>() / total_update_times.len() as f32;
    println!("average total time: {}", average);
    let average = rapier_step_times.iter().sum::<f32>() / rapier_step_times.len() as f32;
    println!("average rapier step time: {}", average);
    println!("total time: {}", timer_total.time());
}
fn pyramid_1_with_height_2() {
    custom_bencher(1000, |app| setup_cubes(app, 1, 2));
}

fn pyramid_2_with_height_20() {
    custom_bencher(100, |app| setup_cubes(app, 2, 20));
}

fn main() {
    pyramid_1_with_height_2();
    pyramid_2_with_height_20();
}
