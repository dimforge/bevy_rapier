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
    configuration.scale = 40.0;

    let mut camera = OrthographicCameraBundle::new_2d();
    camera.transform = Transform::from_translation(Vec3::new(0.0, 30.0, 0.0));
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
        transform: Transform::from_xyz(0.0, 30.0, 0.0),
        ..Default::default()
    });
}

pub fn setup_physics(mut commands: Commands) {
    /*
     * The ground
     */
    let ground_size = 5.0;
    let ground_height = 0.1;

    let collider = ColliderBundle {
        shape: ColliderShape::cuboid(ground_size, ground_height),
        position: [0.0, -ground_height].into(),
        ..Default::default()
    };
    commands
        .spawn_bundle(collider)
        .insert(ColliderPositionSync::Discrete)
        .insert(RapierDebugCollider::default());

    /*
     * A rectangle that only rotate.
     */
    let rigid_body = RigidBodyBundle {
        position: [0.0, 3.0].into(),
        mass_properties: RigidBodyMassPropsFlags::TRANSLATION_LOCKED.into(),
        ..Default::default()
    };
    let collider = ColliderBundle {
        shape: ColliderShape::cuboid(2.0, 0.6),
        ..Default::default()
    };
    commands
        .spawn_bundle(rigid_body)
        .insert_bundle(collider)
        .insert(ColliderPositionSync::Discrete)
        .insert(RapierDebugCollider { color: Color::TEAL });

    /*
     * A tilted cuboid that cannot rotate.
     */
    let rigid_body = RigidBodyBundle {
        position: Isometry2::new([0.3, 5.0].into(), 1.0).into(),
        mass_properties: RigidBodyMassPropsFlags::ROTATION_LOCKED.into(),
        ..Default::default()
    };
    let collider = ColliderBundle {
        shape: ColliderShape::cuboid(0.6, 0.4),
        ..Default::default()
    };
    commands
        .spawn_bundle(rigid_body)
        .insert_bundle(collider)
        .insert(ColliderPositionSync::Discrete)
        .insert(RapierDebugCollider { color: Color::ORANGE })
        .insert(RapierDebugPosition::default())
        // TODO: This isn't working in 2d.
        .insert(RapierDebugPath::default());
}
