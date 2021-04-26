extern crate rapier3d as rapier; // For the debug UI.

use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

use bevy::render::pass::ClearColor;
use rapier3d::pipeline::PhysicsPipeline;
use ui::DebugUiPlugin;

#[path = "../../src_debug_ui/mod.rs"]
mod ui;

#[derive(Default)]
pub struct DespawnResource {
    pub entity: Option<Entity>,
}

fn main() {
    App::build()
        .insert_resource(ClearColor(Color::rgb(
            0xF9 as f32 / 255.0,
            0xF9 as f32 / 255.0,
            0xFF as f32 / 255.0,
        )))
        .insert_resource(Msaa::default())
        .insert_resource(DespawnResource::default())
        .add_plugins(DefaultPlugins)
        .add_plugin(bevy_winit::WinitPlugin::default())
        .add_plugin(bevy_wgpu::WgpuPlugin::default())
        .add_plugin(RapierPhysicsPlugin::<NoUserData>::default())
        .add_plugin(RapierRenderPlugin)
        .add_plugin(DebugUiPlugin)
        .add_startup_system(setup_graphics.system())
        .add_startup_system(setup_physics.system())
        .add_startup_system(enable_physics_profiling.system())
        .add_system(despawn.system())
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
}

pub fn setup_physics(mut commands: Commands, mut despawn: ResMut<DespawnResource>) {
    /*
     * Ground
     */
    let ground_size = 200.1;
    let ground_height = 0.1;

    let collider = ColliderBundle {
        shape: ColliderShape::cuboid(ground_size, ground_height, ground_size),
        position: [0.0, -ground_height, 0.0].into(),
        ..ColliderBundle::default()
    };
    let ground_entity = commands
        .spawn()
        .insert_bundle(collider)
        .insert(ColliderPositionSync::Discrete)
        .insert(ColliderDebugRender::default())
        .id();
    despawn.entity = Some(ground_entity);
    /*
     * Create the cubes
     */
    let num = 8;
    let rad = 1.0;

    let shift = rad * 2.0 + rad;
    let centerx = shift * (num / 2) as f32;
    let centery = shift / 2.0;
    let centerz = shift * (num / 2) as f32;

    let mut offset = -(num as f32) * (rad * 2.0 + rad) * 0.5;
    let mut color = 0;

    for j in 0usize..20 {
        for i in 0..num {
            for k in 0usize..num {
                let x = i as f32 * shift - centerx + offset;
                let y = j as f32 * shift + centery + 3.0;
                let z = k as f32 * shift - centerz + offset;
                color += 1;

                // Build the rigid body.
                let rigid_body = RigidBodyBundle {
                    position: [x, y, z].into(),
                    ..RigidBodyBundle::default()
                };
                let collider = ColliderBundle {
                    shape: ColliderShape::cuboid(rad, rad, rad),
                    ..ColliderBundle::default()
                };
                commands
                    .spawn()
                    .insert_bundle(rigid_body)
                    .insert_bundle(collider)
                    .insert(ColliderPositionSync::Discrete)
                    .insert(ColliderDebugRender::with_id(color));
            }
        }

        offset -= 0.05 * rad * (num as f32 - 1.0);
    }
}

pub fn despawn(mut commands: Commands, time: Res<Time>, mut despawn: ResMut<DespawnResource>) {
    if time.seconds_since_startup() > 5.0 {
        if let Some(entity) = despawn.entity {
            println!("Despawning ground entity");
            commands.entity(entity).despawn();
            despawn.entity = None;
        }
    }
}
