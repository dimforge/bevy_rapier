//! Example using bevy_mod_debugdump to output a graph of systems execution order.
//! run with:
//! `cargo run --example debugdump2 > dump.dot && dot -Tsvg dump.dot > dump.svg`

use bevy::app::PostUpdate;
use bevy::prelude::*;
use bevy_mod_debugdump::{schedule_graph, schedule_graph_dot};
use bevy_rapier2d::prelude::*;

fn main() {
    let mut app = App::new();
    app.add_plugins((
        DefaultPlugins,
        RapierPhysicsPlugin::<NoUserData>::pixels_per_meter(100.0),
        RapierDebugRenderPlugin::default(),
    ));

    let debugdump_settings = schedule_graph::Settings {
        include_system: Some(Box::new(|system| {
            if system.name().starts_with("bevy_pbr")
                || system.name().starts_with("bevy_render")
                || system.name().starts_with("bevy_gizmos")
                || system.name().starts_with("bevy_winit")
                || system.name().starts_with("bevy_sprite")
            {
                return false;
            }
            true
        })),
        ..Default::default()
    };
    let dot = schedule_graph_dot(&mut app, PostUpdate, &debugdump_settings);
    println!("{dot}");
}
