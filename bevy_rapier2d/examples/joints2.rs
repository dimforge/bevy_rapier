extern crate rapier2d as rapier; // For the debug UI.

use bevy::prelude::*;
use bevy_rapier2d::prelude::*;

use bevy::render::pass::ClearColor;
use nalgebra::Point2;
use rapier2d::dynamics::{BallJoint, RigidBodyType};
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

fn setup_graphics(mut commands: Commands, mut configuration: ResMut<RapierConfiguration>) {
    configuration.scale = 12.0;

    let mut camera = OrthographicCameraBundle::new_2d();
    camera.transform = Transform::from_translation(Vec3::new(200.0, -200.0, 0.0));
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
}

pub fn setup_physics(mut commands: Commands) {
    /*
     * Create the balls
     */
    // Build the rigid body.
    // NOTE: a smaller radius (e.g. 0.1) breaks Box2D so
    // in order to be able to compare rapier with Box2D,
    // we set it to 0.4.
    let rad = 0.4;
    let numi = 40; // Num vertical nodes.
    let numk = 40; // Num horizontal nodes.
    let shift = 1.0;

    let mut body_entities = Vec::new();
    let mut color = 0;

    for k in 0..numk {
        for i in 0..numi {
            let fk = k as f32;
            let fi = i as f32;
            color += 1;

            let body_type = if i == 0 && (k % 4 == 0 || k == numk - 1) {
                RigidBodyType::Static
            } else {
                RigidBodyType::Dynamic
            };

            let rigid_body = RigidBodyBundle {
                body_type,
                position: [fk * shift, -fi * shift].into(),
                ..RigidBodyBundle::default()
            };

            let collider = ColliderBundle {
                shape: ColliderShape::cuboid(rad, rad),
                ..ColliderBundle::default()
            };
            let child_entity = commands
                .spawn_bundle(rigid_body)
                .insert_bundle(collider)
                .insert(ColliderPositionSync::Discrete)
                .insert(ColliderDebugRender::with_id(color))
                .id();

            // Vertical joint.
            if i > 0 {
                let parent_entity = *body_entities.last().unwrap();
                let joint = BallJoint::new(Point2::origin(), Point2::new(0.0, shift));
                commands.spawn_bundle((JointBuilderComponent::new(
                    joint,
                    parent_entity,
                    child_entity,
                ),));
            }

            // Horizontal joint.
            if k > 0 {
                let parent_index = body_entities.len() - numi;
                let parent_entity = body_entities[parent_index];
                let joint = BallJoint::new(Point2::origin(), Point2::new(-shift, 0.0));
                commands.spawn_bundle((JointBuilderComponent::new(
                    joint,
                    parent_entity,
                    child_entity,
                ),));
            }

            body_entities.push(child_entity);
        }
    }
}
