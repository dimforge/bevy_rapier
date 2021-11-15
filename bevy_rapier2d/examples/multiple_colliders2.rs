extern crate rapier2d as rapier; // For the debug UI.

use bevy::prelude::*;
use bevy_rapier2d::prelude::*;

use bevy::render::pass::ClearColor;
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
        .run();
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
    let ground_size = 50.0;
    let ground_height = 0.1;

    let collider = ColliderBundle {
        position: [0.0, -ground_height].into(),
        shape: ColliderShape::cuboid(ground_size, ground_height),
        ..Default::default()
    };
    commands
        .spawn_bundle(collider)
        .insert(RapierDebugCollider::default())
        .insert(ColliderPositionSync::Discrete);

    /*
     * Create the cubes
     */
    let num = 4;
    let rad = 0.2;

    let shift = rad * 4.0 + rad;
    let centerx = shift * (num / 2) as f32;
    let centery = shift / 2.0;

    let mut offset = -(num as f32) * (rad * 2.0 + rad) * 0.5;

    for j in 0usize..20 {
        for i in 0..num {
            let x = i as f32 * shift * 5.0 - centerx + offset;
            let y = j as f32 * (shift * 5.0) + centery + 3.0;

            // Build the rigid body.
            let rigid_body = RigidBodyBundle {
                position: [x, y].into(),
                ..Default::default()
            };

            // Attach multiple colliders to this rigid-body using Bevy hierarchy.
            let collider1 = ColliderBundle {
                shape: ColliderShape::cuboid(rad * 10.0, rad),
                ..Default::default()
            };
            let collider2 = ColliderBundle {
                shape: ColliderShape::cuboid(rad, rad * 10.0),
                position: [rad * 10.0, rad * 10.0].into(),
                ..Default::default()
            };
            let collider3 = ColliderBundle {
                shape: ColliderShape::cuboid(rad, rad * 10.0),
                position: [-rad * 10.0, rad * 10.0].into(),
                ..Default::default()
            };

            commands.spawn_bundle(rigid_body).with_children(|parent| {
                parent
                    .spawn_bundle(collider1)
                    .insert(RapierDebugCollider { color: Color::TEAL })
                    .insert(ColliderPositionSync::Discrete);
                parent
                    .spawn_bundle(collider2)
                    .insert(RapierDebugCollider { color: Color::TEAL })
                    .insert(ColliderPositionSync::Discrete);
                parent
                    .spawn_bundle(collider3)
                    .insert(RapierDebugCollider { color: Color::TEAL })
                    .insert(ColliderPositionSync::Discrete);
            });
        }

        offset -= 0.05 * rad * (num as f32 - 1.0);
    }
}
