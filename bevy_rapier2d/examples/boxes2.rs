extern crate rapier2d as rapier; // For the debug UI.

use bevy::prelude::*;
use bevy_rapier2d::prelude::*;

use bevy::render::pass::ClearColor;
use nalgebra::Isometry2;
use rapier2d::pipeline::PhysicsPipeline;
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
        .add_plugin(RapierPhysicsPlugin::<NoUserData>::default())
        .add_plugin(RapierDebugPlugin)
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

    let mut camera = OrthographicCameraBundle::new_2d();
    camera.transform = Transform::from_translation(Vec3::new(0.0, 200.0, 0.0));
    commands.spawn_bundle(LightBundle {
        transform: Transform::from_translation(Vec3::new(1000.0, 10.0, 2000.0)),
        light: Light {
            intensity: 100_000_000_.0,
            range: 6000.0,
            ..Default::default()
        },
        ..Default::default()
    });
    commands.spawn_bundle(camera);
    commands.spawn_bundle(RapierDebugOrthographicCameraBundle {
        transform: Transform::from_xyz(0.0, 200.0, 0.0),
        ..Default::default()
    });
}

pub fn setup_physics(mut commands: Commands) {
    /*
     * Ground
     */
    let ground_size = 25.0;

    let collider = ColliderBundle {
        shape: ColliderShape::cuboid(ground_size, 1.0),
        ..Default::default()
    };
    commands
        .spawn_bundle(collider)
        .insert(RapierDebugCollider::default())
        .insert(ColliderPositionSync::Discrete);

    let collider = ColliderBundle {
        shape: ColliderShape::cuboid(ground_size * 2.0, 1.2),
        position: Isometry2::new(
            [ground_size, ground_size * 2.0].into(),
            std::f32::consts::FRAC_PI_2,
        )
        .into(),
        ..Default::default()
    };
    commands
        .spawn_bundle(collider)
        .insert(RapierDebugCollider::default())
        .insert(ColliderPositionSync::Discrete);

    let collider = ColliderBundle {
        shape: ColliderShape::cuboid(ground_size * 2.0, 1.2),
        position: Isometry2::new(
            [-ground_size, ground_size * 2.0].into(),
            std::f32::consts::FRAC_PI_2,
        )
        .into(),
        ..Default::default()
    };
    commands
        .spawn_bundle(collider)
        .insert(RapierDebugCollider::default())
        .insert(ColliderPositionSync::Discrete);

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
            let body = RigidBodyBundle {
                position: [x, y].into(),
                ..Default::default()
            };
            let collider = ColliderBundle {
                shape: ColliderShape::cuboid(rad, rad),
                ..Default::default()
            };
            commands
                .spawn_bundle(body)
                .insert_bundle(collider)
                .insert(RapierDebugCollider { color: Color::VIOLET })
                .insert(ColliderPositionSync::Discrete);
        }
    }
}
