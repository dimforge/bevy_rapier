extern crate rapier2d as rapier; // For the debug UI.

use bevy::prelude::*;
use bevy::render::pass::ClearColor;
use bevy_rapier2d::physics::{RapierConfiguration, RapierPhysicsPlugin};
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
        .run();
}

fn enable_physics_profiling(mut pipeline: ResMut<PhysicsPipeline>) {
    pipeline.counters.enable()
}

fn setup_graphics(mut commands: Commands, mut configuration: ResMut<RapierConfiguration>) {
    configuration.scale = 10.0;

    commands
        .spawn(LightComponents {
            transform: Transform::from_translation(Vec3::new(1000.0, 100.0, 2000.0)),
            ..Default::default()
        })
        .spawn(Camera2dComponents {
            transform: Transform::from_translation(Vec3::new(0.0, 200.0, 0.0)),
            ..Camera2dComponents::default()
        });
}

pub fn setup_physics(mut commands: Commands) {
    /*
     * Ground
     */
    let ground_size = 25.0;

    let rigid_body = RigidBodyBuilder::new_static();
    let collider = ColliderBuilder::cuboid(ground_size, 1.2);
    commands.spawn((rigid_body, collider));

    let rigid_body = RigidBodyBuilder::new_static()
        .rotation(std::f32::consts::FRAC_PI_2)
        .translation(ground_size, ground_size * 2.0);
    let collider = ColliderBuilder::cuboid(ground_size * 2.0, 1.2);
    commands.spawn((rigid_body, collider));

    let body = RigidBodyBuilder::new_static()
        .rotation(std::f32::consts::FRAC_PI_2)
        .translation(-ground_size, ground_size * 2.0);
    let collider = ColliderBuilder::cuboid(ground_size * 2.0, 1.2);
    commands.spawn((body, collider));

    /*
     * Create the cubes
     */
    let num = 20;
    let rad = 0.5;

    let shift = rad * 2.0;
    let centerx = shift * (num as f32) / 2.0;
    let centery = shift / 2.0;

    for i in 0..num {
        for j in 0usize..num * 5 {
            let x = i as f32 * shift - centerx;
            let y = j as f32 * shift + centery + 2.0;

            // Build the rigid body.
            let body = RigidBodyBuilder::new_dynamic().translation(x, y);
            let collider = ColliderBuilder::cuboid(rad, rad).density(1.0);
            commands.spawn((body, collider));
        }
    }
}
