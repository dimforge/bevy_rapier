//! Example for RapierContext serialization, run with `--features serde-serialize`.

use bevy::prelude::*;
use bevy_rapier2d::prelude::*;

/// Note: This will end up in duplication for testbed, but that's more simple.
mod joints2;

fn main() {
    App::new()
        .insert_resource(ClearColor(Color::srgb(
            0xF9 as f32 / 255.0,
            0xF9 as f32 / 255.0,
            0xFF as f32 / 255.0,
        )))
        .add_plugins((
            DefaultPlugins,
            RapierPhysicsPlugin::<NoUserData>::pixels_per_meter(100.0),
            RapierDebugRenderPlugin::default(),
        ))
        .add_systems(Startup, (joints2::setup_graphics, joints2::setup_physics))
        .add_systems(PostUpdate, print_physics)
        .add_systems(Last, quit)
        .run();
}

pub fn print_physics(context: Res<RapierContext>) {
    println!(
        "{}",
        serde_json::to_string_pretty(&(*context)).expect("Unable to serialize `RapierContext`")
    );
}

fn quit(mut exit_event: EventWriter<AppExit>) {
    exit_event.send(AppExit::Success);
}
