extern crate rapier2d as rapier; // For the debug UI.

use bevy::prelude::*;
use bevy::render::pass::ClearColor;
use bevy_rapier2d::physics::{JointBuilderComponent, RapierConfiguration, RapierPhysicsPlugin};
use bevy_rapier2d::render::RapierRenderPlugin;
use nalgebra::Point2;
use rapier::dynamics::{BallJoint, BodyStatus};
use rapier2d::dynamics::RigidBodyBuilder;
use rapier2d::geometry::ColliderBuilder;
use rapier2d::pipeline::PhysicsPipeline;
use ui::DebugUiPlugin;

#[path = "../../src_debug_ui/mod.rs"]
mod ui;

#[derive(Default)]
pub struct DespawnResource {
    entities: Vec<Entity>,
}

fn main() {
    App::build()
        .add_resource(ClearColor(Color::rgb(
            0xF9 as f32 / 255.0,
            0xF9 as f32 / 255.0,
            0xFF as f32 / 255.0,
        )))
        .add_resource(Msaa::default())
        .add_resource(DespawnResource::default())
        .add_plugins(DefaultPlugins)
        .add_plugin(bevy_winit::WinitPlugin::default())
        .add_plugin(bevy_wgpu::WgpuPlugin::default())
        .add_plugin(RapierPhysicsPlugin)
        .add_plugin(RapierRenderPlugin)
        .add_plugin(DebugUiPlugin)
        .add_startup_system(setup_graphics)
        .add_startup_system(setup_physics)
        .add_startup_system(enable_physics_profiling)
        .add_system(despawn)
        .run();
}

fn enable_physics_profiling(mut pipeline: ResMut<PhysicsPipeline>) {
    pipeline.counters.enable()
}

fn setup_graphics(commands: &mut Commands, mut configuration: ResMut<RapierConfiguration>) {
    configuration.scale = 12.0;

    commands
        .spawn(LightBundle {
            transform: Transform::from_translation(Vec3::new(1000.0, 100.0, 2000.0)),
            ..Default::default()
        })
        .spawn(Camera2dBundle {
            transform: Transform::from_translation(Vec3::new(200.0, -200.0, 0.0)),
            ..Camera2dBundle::default()
        });
}

pub fn setup_physics(commands: &mut Commands, mut despawn: ResMut<DespawnResource>) {
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

    for k in 0..numk {
        for i in 0..numi {
            let fk = k as f32;
            let fi = i as f32;

            let status = if i == 0 && (k % 4 == 0 || k == numk - 1) {
                BodyStatus::Static
            } else {
                BodyStatus::Dynamic
            };

            let rigid_body = RigidBodyBuilder::new(status).translation(fk * shift, -fi * shift);
            let collider = ColliderBuilder::cuboid(rad, rad).density(1.0);
            let child_entity = commands
                .spawn((rigid_body, collider))
                .current_entity()
                .unwrap();

            // Vertical joint.
            if i > 0 {
                let parent_entity = *body_entities.last().unwrap();
                let joint = BallJoint::new(Point2::origin(), Point2::new(0.0, shift));
                let entity = commands
                    .spawn((JointBuilderComponent::new(
                        joint,
                        parent_entity,
                        child_entity,
                    ),))
                    .current_entity()
                    .expect("Failed to spawn joint");
                if i == (numi / 2) || (k % 4 == 0 || k == numk - 1) {
                    despawn.entities.push(entity);
                }
            }

            // Horizontal joint.
            if k > 0 {
                let parent_index = body_entities.len() - numi;
                let parent_entity = body_entities[parent_index];
                let joint = BallJoint::new(Point2::origin(), Point2::new(-shift, 0.0));
                let entity = commands
                    .spawn((JointBuilderComponent::new(
                        joint,
                        parent_entity,
                        child_entity,
                    ),))
                    .current_entity()
                    .expect("Failed to spawn joint");
                if i == (numi / 2) || (k % 4 == 0 || k == numk - 1) {
                    despawn.entities.push(entity);
                }
            }

            body_entities.push(child_entity);
        }
    }
}

pub fn despawn(commands: &mut Commands, time: Res<Time>, mut despawn: ResMut<DespawnResource>) {
    if time.seconds_since_startup() > 10.0 {
        for entity in &despawn.entities {
            println!("Despawning joint entity");
            commands.despawn(*entity);
        }
        despawn.entities.clear();
    }
}
