extern crate rapier3d as rapier; // For the debug UI.

use bevy::prelude::*;
use bevy::render::pass::ClearColor;
use bevy_rapier3d::physics::RapierPhysicsPlugin;
use bevy_rapier3d::render::RapierRenderPlugin;
use nalgebra::Vector3;
use rapier3d::dynamics::RigidBodyBuilder;
use rapier3d::geometry::ColliderBuilder;
use rapier3d::pipeline::PhysicsPipeline;
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
        .add_plugins(DefaultPlugins)
        .add_plugin(bevy_winit::WinitPlugin::default())
        .add_plugin(bevy_wgpu::WgpuPlugin::default())
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

fn setup_graphics(commands: &mut Commands) {
    commands
        .spawn(LightBundle {
            transform: Transform::from_translation(Vec3::new(1000.0, 100.0, 2000.0)),
            ..Default::default()
        })
        .spawn(Camera3dBundle {
            transform: Transform::from_matrix(Mat4::face_toward(
                Vec3::new(10.0, 3.0, 0.0),
                Vec3::new(0.0, 3.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
            )),
            ..Default::default()
        });
}

pub fn setup_physics(commands: &mut Commands) {
    /*
     * The ground
     */
    let ground_size = 5.0;
    let ground_height = 0.1;

    let rigid_body = RigidBodyBuilder::new_static().translation(0.0, -ground_height, 0.0);
    let collider = ColliderBuilder::cuboid(ground_size, ground_height, ground_size);
    commands.spawn((rigid_body, collider));

    /*
     * A rectangle that only rotates along the `x` axis.
     */
    let rigid_body = RigidBodyBuilder::new_dynamic()
        .translation(0.0, 3.0, 0.0)
        .lock_translations()
        .principal_angular_inertia(Vector3::zeros(), Vector3::new(true, false, false));
    let collider = ColliderBuilder::cuboid(0.2, 0.6, 2.0);
    commands.spawn((rigid_body, collider));

    /*
     * A tilted cuboid that cannot rotate.
     */
    let rigid_body = RigidBodyBuilder::new_dynamic()
        .translation(0.0, 5.0, 0.0)
        .rotation(Vector3::x() * 1.0)
        .lock_rotations();
    let collider = ColliderBuilder::cuboid(0.6, 0.4, 0.4);
    commands.spawn((rigid_body, collider));
}
