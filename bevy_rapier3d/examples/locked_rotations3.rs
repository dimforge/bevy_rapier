extern crate rapier3d as rapier; // For the debug UI.

use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

use bevy::render::pass::ClearColor;
use rapier::dynamics::RigidBodyMassPropsFlags;
use rapier::geometry::ColliderShape;
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
            Vec3::new(10.0, 3.0, 0.0),
            Vec3::new(0.0, 3.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        )),
        ..Default::default()
    });
    commands.spawn_bundle(RapierDebugPerspectiveCameraBundle {
        transform: Transform::from_matrix(Mat4::face_toward(
            Vec3::new(10.0, 3.0, 0.0),
            Vec3::new(0.0, 3.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        )),
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
        shape: ColliderShape::cuboid(ground_size, ground_height, ground_size),
        position: [0.0, -ground_height, 0.0].into(),
        ..ColliderBundle::default()
    };
    commands
        .spawn_bundle(collider)
        .insert(ColliderDebugRender::default())
        .insert(ColliderPositionSync::Discrete);

    /*
     * A rectangle that only rotates along the `x` axis.
     */
    let locked_dofs = RigidBodyMassPropsFlags::TRANSLATION_LOCKED
        | RigidBodyMassPropsFlags::ROTATION_LOCKED_Y
        | RigidBodyMassPropsFlags::ROTATION_LOCKED_Z;

    let rigid_body = RigidBodyBundle {
        position: [0.0, 3.0, 0.0].into(),
        mass_properties: locked_dofs.into(),
        ..RigidBodyBundle::default()
    };

    let collider = ColliderBundle {
        shape: ColliderShape::cuboid(0.2, 0.6, 2.0),
        ..ColliderBundle::default()
    };
    commands
        .spawn_bundle(rigid_body)
        .insert_bundle(collider)
        .insert(ColliderDebugRender::with_id(0))
        .insert(ColliderPositionSync::Discrete);

    /*
     * A tilted cuboid that cannot rotate.
     */
    let rigid_body = RigidBodyBundle {
        position: (Vec3::new(0.0, 5.0, 0.0), Quat::from_rotation_x(1.0)).into(),
        mass_properties: RigidBodyMassPropsFlags::ROTATION_LOCKED.into(),
        ..RigidBodyBundle::default()
    };

    let collider = ColliderBundle {
        shape: ColliderShape::cuboid(0.6, 0.4, 0.4),
        ..ColliderBundle::default()
    };

    commands
        .spawn_bundle(rigid_body)
        .insert_bundle(collider)
        .insert(ColliderDebugRender::with_id(1))
        .insert(ColliderPositionSync::Discrete);
}
