extern crate rapier3d as rapier; // For the debug UI.

use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

use bevy::render::pass::ClearColor;
use rapier::geometry::ColliderShape;
use rapier::pipeline::PhysicsPipeline;
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

fn setup_graphics(mut commands: Commands) {
    commands.spawn_bundle(LightBundle {
        transform: Transform::from_translation(Vec3::new(100.0, 10.0, 200.0)),
        light: Light {
            intensity: 100_000.0,
            range: 3000.0,
            ..Default::default()
        },
        ..Default::default()
    });
    commands.spawn_bundle(PerspectiveCameraBundle {
        transform: Transform::from_matrix(Mat4::face_toward(
            Vec3::new(-30.0, 30.0, 100.0),
            Vec3::new(0.0, 10.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        )),
        ..Default::default()
    });
    commands.spawn_bundle(RapierDebugPerspectiveCameraBundle {
        transform: Transform::from_matrix(Mat4::face_toward(
            Vec3::new(-30.0, 30.0, 100.0),
            Vec3::new(0.0, 10.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        )),
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
        shape: ColliderShape::cuboid(ground_size, ground_height, ground_size),
        position: [0.0, -ground_height, 0.0].into(),
        ..ColliderBundle::default()
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
    let centerz = shift * (num / 2) as f32;

    let mut offset = -(num as f32) * (rad * 2.0 + rad) * 0.5;

    for j in 0usize..20 {
        for i in 0..num {
            for k in 0usize..num {
                let x = i as f32 * shift * 5.0 - centerx + offset;
                let y = j as f32 * (shift * 5.0) + centery + 3.0;
                let z = k as f32 * shift * 2.0 - centerz + offset;

                // Build the rigid body.
                let rigid_body = RigidBodyBundle {
                    position: [x, y, z].into(),
                    ..RigidBodyBundle::default()
                };

                // Attach multiple colliders to this rigid-body using Bevy hierarchy.
                let collider1 = ColliderBundle {
                    shape: ColliderShape::cuboid(rad * 10.0, rad, rad),
                    ..ColliderBundle::default()
                };
                let collider2 = ColliderBundle {
                    shape: ColliderShape::cuboid(rad, rad * 10.0, rad),
                    position: [rad * 10.0, rad * 10.0, 0.0].into(),
                    ..ColliderBundle::default()
                };
                let collider3 = ColliderBundle {
                    shape: ColliderShape::cuboid(rad, rad * 10.0, rad),
                    position: [-rad * 10.0, rad * 10.0, 0.0].into(),
                    ..ColliderBundle::default()
                };

                let sync = ColliderPositionSync::Discrete;

                commands
                    .spawn()
                    .insert_bundle(rigid_body)
                    .with_children(|parent| {
                        parent
                            .spawn()
                            .insert_bundle(collider1)
                            .insert_bundle((RapierDebugCollider { color: Color::BLUE }, sync));
                        parent
                            .spawn()
                            .insert_bundle(collider2)
                            .insert_bundle((RapierDebugCollider { color: Color::BLUE }, sync));
                        parent
                            .spawn()
                            .insert_bundle(collider3)
                            .insert_bundle((RapierDebugCollider { color: Color::BLUE }, sync));
                    });
            }
        }

        offset -= 0.05 * rad * (num as f32 - 1.0);
    }
}
