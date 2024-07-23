//! Translated from rapier benchmark.
//!
//! <https://github.com/dimforge/rapier/blob/87ada34008f4a1a313ccf8c3040040bab4f10e69/benchmarks3d/many_pyramids3.rs>

pub mod common;
pub mod pyramids;

use common::default_app;
use common::wait_app_start;

use bevy::prelude::*;
use bevy_rapier3d::plugin::RapierContext;

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
        let rc = app
            .world_mut()
            .query::<&RapierContext>()
            .single(app.world());
        rapier_step_times.push(rc.pipeline.counters.step_time.time() as f32);
        total_update_times.push(elapsed_time);
    }
    timer_total.pause();
    let average_total = total_update_times.iter().sum::<f32>() / total_update_times.len() as f32;
    println!("average total time: {} ms", average_total);
    let average_rapier_step =
        rapier_step_times.iter().sum::<f32>() / rapier_step_times.len() as f32;
    println!("average rapier step time: {} ms", average_rapier_step);
    let average_rapier_overhead = average_total - average_rapier_step;
    println!("average bevy overhead: {} ms", average_rapier_overhead);
    println!("total time: {} ms", timer_total.time());
}
