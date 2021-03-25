extern crate rapier3d as rapier; // For the debug UI.

use bevy::prelude::*;
use bevy::render::pass::ClearColor;
use bevy_rapier3d::physics::{EventQueue, RapierPhysicsPlugin};
use bevy_rapier3d::render::RapierRenderPlugin;
use rapier3d::dynamics::RigidBodyBuilder;
use rapier3d::geometry::ColliderBuilder;
use rapier3d::pipeline::PhysicsPipeline;
use ui::DebugUiPlugin;

#[path = "../../src_debug_ui/mod.rs"]
mod ui;

fn main() {
    App::build()
        .insert_resource(ClearColor(Color::rgb(
            0xF9 as f32 / 255.0,
            0xF9 as f32 / 255.0,
            0xFF as f32 / 255.0,
        )))
        .insert_resource(Msaa::default())
        .add_plugins(DefaultPlugins)
        .add_plugin(bevy_winit::WinitPlugin::default())
        .add_plugin(bevy_wgpu::WgpuPlugin::default())
        .add_plugin(RapierPhysicsPlugin)
        .add_plugin(RapierRenderPlugin)
        .add_plugin(DebugUiPlugin)
        .add_startup_system(setup_graphics.system())
        .add_startup_system(setup_physics.system())
        .add_startup_system(enable_physics_profiling.system())
        .add_system_to_stage(CoreStage::PostUpdate, display_events.system())
        .run();
}

fn enable_physics_profiling(mut pipeline: ResMut<PhysicsPipeline>) {
    pipeline.counters.enable()
}

fn setup_graphics(mut commands: Commands) {
    commands.spawn().insert_bundle(LightBundle {
        transform: Transform::from_translation(Vec3::new(1000.0, 100.0, 2000.0)),
        ..Default::default()
    });
    commands.spawn().insert_bundle(PerspectiveCameraBundle {
        transform: Transform::from_matrix(Mat4::face_toward(
            Vec3::new(0.0, 0.0, 25.0),
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        )),
        ..Default::default()
    });
}

fn display_events(events: Res<EventQueue>) {
    while let Ok(intersection_event) = events.intersection_events.pop() {
        println!("Received intersection event: {:?}", intersection_event);
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
    let collider = ColliderBuilder::cuboid(4.0, 1.2, 1.2);
    commands.spawn().insert_bundle((rigid_body, collider));

    let rigid_body = RigidBodyBuilder::new_static().translation(0.0, 5.0, 0.0);
    let collider = ColliderBuilder::cuboid(4.0, 1.2, 1.0).sensor(true);
    commands.spawn().insert_bundle((rigid_body, collider));

    let rigid_body = RigidBodyBuilder::new_dynamic().translation(0.0, 13.0, 0.0);
    let collider = ColliderBuilder::cuboid(0.5, 0.5, 0.5);
    commands.spawn().insert_bundle((rigid_body, collider));
}
