extern crate rapier3d as rapier; // For the debug UI.

use bevy::prelude::*;
use bevy::render::pass::ClearColor;
use bevy_rapier3d::physics::RapierPhysicsPlugin;
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
        .add_system(platform_move.system())
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
        .spawn(PerspectiveCameraBundle {
            transform: Transform::from_matrix(Mat4::face_toward(
                Vec3::new(0.0, 0.0, 25.0),
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(0.0,2.0, 0.0),
            )),
            ..Default::default()
        });
}

struct Platform(bool);

pub fn setup_physics(
    commands: &mut Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn((
        RigidBodyBuilder::new_kinematic(),
        ColliderBuilder::cuboid(1.2, 1.2, 1.2),
    )).with(Platform(true));

    commands.spawn((
        RigidBodyBuilder::new_dynamic().translation(0.0, 6.0, 0.0),
        ColliderBuilder::cuboid(0.5, 0.5, 0.5)
    ));

    // RIGHT: modify existing, and insert new
    commands.spawn(PbrBundle {
        mesh: meshes.add(Mesh::from(shape::Cube { size: 1.2 })),
        material: materials.add(Color::rgb(0.8, 0.7, 0.6).into()),
        transform: Transform::from_translation(Vec3::new(3., 1., 0.)),
        ..Default::default()
    }).with(Platform(true));

    commands.insert(commands.current_entity().unwrap(), (
        RigidBodyBuilder::new_kinematic(),
        ColliderBuilder::cuboid(4.0, 1.2, 1.2),
    ));

    commands.spawn((
        RigidBodyBuilder::new_dynamic().translation(3.0, 6.0, 0.0),
        ColliderBuilder::cuboid(0.5, 0.5, 0.5),
    ));
}

fn platform_move(mut platforms: Query<(&mut Transform, &mut Platform), With<Platform>>) {
    for (mut transform, mut platform) in platforms.iter_mut() {
        transform.translation.y += if platform.0 { 1. } else { -1. } * 0.15;
        if transform.translation.y > 3.0 {
            platform.0 = false;
        } else if transform.translation.y < 1.0 {
            platform.0 = true;
        }
    }
}
