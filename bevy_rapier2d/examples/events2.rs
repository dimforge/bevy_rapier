extern crate rapier2d as rapier; // For the debug UI.

use bevy::prelude::*;
use bevy::render::pass::ClearColor;
use bevy_rapier2d::physics::{EventQueue, RapierConfiguration, RapierPhysicsPlugin};
use bevy_rapier2d::render::RapierRenderPlugin;
use rapier2d::dynamics::RigidBodyBuilder;
use rapier2d::geometry::ColliderBuilder;
use rapier2d::pipeline::PhysicsPipeline;
use ui::DebugUiPlugin;

#[path = "../../src_debug_ui/mod.rs"]
mod ui;

fn main() {
    App::build()
        .add_resource(ClearColor(Color::rgb(
            0xF9 as f32 / 255.0,
            0xF9 as f32 / 255.0,
            0xFF as f32 / 255.0,
        )))
        .add_resource(Msaa::default())
        .add_default_plugins()
        .add_plugin(RapierPhysicsPlugin)
        .add_plugin(RapierRenderPlugin)
        .add_plugin(DebugUiPlugin)
        .add_startup_system(setup_graphics.system())
        .add_startup_system(setup_physics.system())
        .add_startup_system(enable_physics_profiling.system())
        .add_system_to_stage(stage::POST_UPDATE, display_events.system())
        .run();
}

fn enable_physics_profiling(mut pipeline: ResMut<PhysicsPipeline>) {
    pipeline.counters.enable()
}

fn setup_graphics(mut commands: Commands, mut configuration: ResMut<RapierConfiguration>) {
    configuration.scale = 15.0;

    commands
        .spawn(LightComponents {
            transform: Transform::from_translation(Vec3::new(1000.0, 100.0, 2000.0)),
            ..Default::default()
        })
        .spawn(Camera2dComponents {
            transform: Transform::from_translation(Vec3::new(0.0, 0.0, 0.0)),
            ..Camera2dComponents::default()
        });
}

fn display_events(events: Res<EventQueue>) {
    while let Ok(proximity_event) = events.proximity_events.pop() {
        println!("Received proximity event: {:?}", proximity_event);
    }

    while let Ok(contact_event) = events.contact_events.pop() {
        println!("Received contact event: {:?}", contact_event);
    }
}

pub fn setup_physics(mut commands: Commands) {
    /*
     * Ground
     */
    let rigid_body = RigidBodyBuilder::new_static();
    let collider = ColliderBuilder::cuboid(4.0, 1.2);
    commands.spawn((rigid_body, collider));

    let rigid_body = RigidBodyBuilder::new_static().translation(0.0, 5.0);
    let collider = ColliderBuilder::cuboid(4.0, 1.2).sensor(true);
    commands.spawn((rigid_body, collider));

    let rigid_body = RigidBodyBuilder::new_dynamic().translation(0.0, 13.0);
    let collider = ColliderBuilder::cuboid(0.5, 0.5);
    commands.spawn((rigid_body, collider));
}
