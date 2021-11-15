extern crate rapier2d as rapier; // For the debug UI.

use bevy::prelude::*;
use bevy_rapier2d::prelude::*;

use bevy::render::pass::ClearColor;
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
        .add_system_to_stage(CoreStage::PostUpdate, display_events.system())
        .run();
}

fn enable_physics_profiling(mut pipeline: ResMut<PhysicsPipeline>) {
    pipeline.counters.enable()
}

fn setup_graphics(mut commands: Commands, mut configuration: ResMut<RapierConfiguration>) {
    configuration.scale = 15.0;

    commands.spawn_bundle(LightBundle {
        transform: Transform::from_translation(Vec3::new(1000.0, 10.0, 2000.0)),
        light: Light {
            intensity: 100_000_000_.0,
            range: 6000.0,
            ..Default::default()
        },
        ..Default::default()
    });
    commands
        .spawn()
        .insert_bundle(OrthographicCameraBundle::new_2d());
    commands
        .spawn()
        .insert_bundle(RapierDebugOrthographicCameraBundle::default());
}

fn display_events(
    mut intersection_events: EventReader<IntersectionEvent>,
    mut contact_events: EventReader<ContactEvent>,
) {
    for intersection_event in intersection_events.iter() {
        println!("Received intersection event: {:?}", intersection_event);
    }

    for contact_event in contact_events.iter() {
        println!("Received contact event: {:?}", contact_event);
    }
}

pub fn setup_physics(mut commands: Commands) {
    /*
     * Ground
     */
    let collider = ColliderBundle {
        shape: ColliderShape::cuboid(4.0, 1.2),
        ..Default::default()
    };
    commands
        .spawn_bundle(collider)
        .insert(ColliderPositionSync::Discrete)
        .insert(RapierDebugCollider::default());

    let collider = ColliderBundle {
        shape: ColliderShape::cuboid(4.0, 1.2),
        collider_type: ColliderType::Sensor,
        position: [0.0, 5.0].into(),
        ..Default::default()
    };
    commands
        .spawn_bundle(collider)
        .insert(ColliderPositionSync::Discrete)
        .insert(RapierDebugCollider { color: Color::TEAL });

    let rigid_body = RigidBodyBundle {
        position: [0.0, 13.0].into(),
        ..Default::default()
    };
    let collider = ColliderBundle {
        shape: ColliderShape::cuboid(0.5, 0.5),
        flags: (ActiveEvents::INTERSECTION_EVENTS | ActiveEvents::CONTACT_EVENTS).into(),
        ..Default::default()
    };
    commands
        .spawn_bundle(rigid_body)
        .insert_bundle(collider)
        .insert(ColliderPositionSync::Discrete)
        .insert(RapierDebugCollider { color: Color::TEAL });
}
