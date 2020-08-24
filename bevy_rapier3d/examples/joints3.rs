extern crate nalgebra as na;
extern crate rapier3d as rapier; // For the debug UI.

use bevy::prelude::*;
use bevy::render::pass::ClearColor;
use bevy_rapier3d::physics::{JointBuilderComponent, RapierPhysicsPlugin};
use bevy_rapier3d::render::RapierRenderPlugin;
use na::{Isometry3, Point3, Unit, Vector3};
use rapier::dynamics::{BallJoint, BodyStatus, FixedJoint, PrismaticJoint, RevoluteJoint};
use rapier3d::dynamics::RigidBodyBuilder;
use rapier3d::geometry::ColliderBuilder;
use rapier3d::pipeline::PhysicsPipeline;
use ui::DebugUiPlugin;

#[path = "../../src_debug_ui/mod.rs"]
mod ui;

fn main() {
    App::build()
        .add_resource(ClearColor(Color::rgb(
            0xF9 as f32 / 255.0,
            0xF9 as f32 / 255.0,
            0xFF as f32 / 255.0,
        )))
        .add_resource(Msaa { samples: 2 })
        .add_default_plugins()
        .add_plugin(RapierPhysicsPlugin)
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

fn setup_graphics(mut commands: Commands) {
    commands
        .spawn(LightComponents {
            translation: Translation::new(1000.0, 100.0, 2000.0),
            ..Default::default()
        })
        .spawn(Camera3dComponents {
            transform: Transform::new_sync_disabled(Mat4::face_toward(
                Vec3::new(15.0, 5.0, 42.0),
                Vec3::new(13.0, 1.0, 1.0),
                Vec3::new(0.0, 1.0, 0.0),
            )),
            ..Default::default()
        });
}

fn create_prismatic_joints(commands: &mut Commands, origin: Point3<f32>, num: usize) {
    let rad = 0.4;
    let shift = 1.0;

    let ground = RigidBodyBuilder::new_static().translation(origin.x, origin.y, origin.z);
    let collider = ColliderBuilder::cuboid(rad, rad, rad);
    let mut curr_parent = commands.spawn((ground, collider)).current_entity().unwrap();

    for i in 0..num {
        let z = origin.z + (i + 1) as f32 * shift;
        let density = 1.0;
        let rigid_body = RigidBodyBuilder::new_dynamic().translation(origin.x, origin.y, z);
        let collider = ColliderBuilder::cuboid(rad, rad, rad).density(density);
        let curr_child = commands
            .spawn((rigid_body, collider))
            .current_entity()
            .unwrap();

        let axis = if i % 2 == 0 {
            Unit::new_normalize(Vector3::new(1.0, 1.0, 0.0))
        } else {
            Unit::new_normalize(Vector3::new(-1.0, 1.0, 0.0))
        };

        let z = Vector3::z();
        let mut prism = PrismaticJoint::new(
            Point3::origin(),
            axis,
            z,
            Point3::new(0.0, 0.0, -shift),
            axis,
            z,
        );
        prism.limits_enabled = true;
        prism.limits[0] = -2.0;
        prism.limits[1] = 2.0;

        commands.spawn((JointBuilderComponent::new(prism, curr_parent, curr_child),));

        curr_parent = curr_child;
    }
}

fn create_revolute_joints(commands: &mut Commands, origin: Point3<f32>, num: usize) {
    let rad = 0.4;
    let shift = 2.0;

    let ground = RigidBodyBuilder::new_static().translation(origin.x, origin.y, 0.0);
    let collider = ColliderBuilder::cuboid(rad, rad, rad);
    let mut curr_parent = commands.spawn((ground, collider)).current_entity().unwrap();

    for i in 0..num {
        // Create four bodies.
        let z = origin.z + i as f32 * shift * 2.0 + shift;
        let positions = [
            Isometry3::translation(origin.x, origin.y, z),
            Isometry3::translation(origin.x + shift, origin.y, z),
            Isometry3::translation(origin.x + shift, origin.y, z + shift),
            Isometry3::translation(origin.x, origin.y, z + shift),
        ];

        let mut handles = [curr_parent; 4];
        for k in 0..4 {
            let density = 1.0;
            let rigid_body = RigidBodyBuilder::new_dynamic().position(positions[k]);
            let collider = ColliderBuilder::cuboid(rad, rad, rad).density(density);
            handles[k] = commands
                .spawn((rigid_body, collider))
                .current_entity()
                .unwrap();
        }

        // Setup four joints.
        let o = Point3::origin();
        let x = Vector3::x_axis();
        let z = Vector3::z_axis();

        let revs = [
            RevoluteJoint::new(o, z, Point3::new(0.0, 0.0, -shift), z),
            RevoluteJoint::new(o, x, Point3::new(-shift, 0.0, 0.0), x),
            RevoluteJoint::new(o, z, Point3::new(0.0, 0.0, -shift), z),
            RevoluteJoint::new(o, x, Point3::new(shift, 0.0, 0.0), x),
        ];

        commands.spawn((JointBuilderComponent::new(revs[0], curr_parent, handles[0]),));
        commands.spawn((JointBuilderComponent::new(revs[1], handles[0], handles[1]),));
        commands.spawn((JointBuilderComponent::new(revs[2], handles[1], handles[2]),));
        commands.spawn((JointBuilderComponent::new(revs[3], handles[2], handles[3]),));

        curr_parent = handles[3];
    }
}

fn create_fixed_joints(commands: &mut Commands, origin: Point3<f32>, num: usize) {
    let rad = 0.4;
    let shift = 1.0;

    let mut body_entities = Vec::new();

    for k in 0..num {
        for i in 0..num {
            let fk = k as f32;
            let fi = i as f32;

            // NOTE: the num - 2 test is to avoid two consecutive
            // fixed bodies. Because physx will crash if we add
            // a joint between these.
            let status = if i == 0 && (k % 4 == 0 && k != num - 2 || k == num - 1) {
                BodyStatus::Static
            } else {
                BodyStatus::Dynamic
            };

            let rigid_body = RigidBodyBuilder::new(status).translation(
                origin.x + fk * shift,
                origin.y,
                origin.z + fi * shift,
            );
            let collider = ColliderBuilder::ball(rad).density(1.0);
            let child_entity = commands
                .spawn((rigid_body, collider))
                .current_entity()
                .unwrap();

            // Vertical joint.
            if i > 0 {
                let parent_entity = *body_entities.last().unwrap();
                let joint = FixedJoint::new(
                    Isometry3::identity(),
                    Isometry3::translation(0.0, 0.0, -shift),
                );
                commands.spawn((JointBuilderComponent::new(
                    joint,
                    parent_entity,
                    child_entity,
                ),));
            }

            // Horizontal joint.
            if k > 0 {
                let parent_index = body_entities.len() - num;
                let parent_entity = body_entities[parent_index];
                let joint = FixedJoint::new(
                    Isometry3::identity(),
                    Isometry3::translation(-shift, 0.0, 0.0),
                );
                commands.spawn((JointBuilderComponent::new(
                    joint,
                    parent_entity,
                    child_entity,
                ),));
            }

            body_entities.push(child_entity);
        }
    }
}

fn create_ball_joints(commands: &mut Commands, num: usize) {
    let rad = 0.4;
    let shift = 1.0;

    let mut body_entities = Vec::new();

    for k in 0..num {
        for i in 0..num {
            let fk = k as f32;
            let fi = i as f32;

            let status = if i == 0 && (k % 4 == 0 || k == num - 1) {
                BodyStatus::Static
            } else {
                BodyStatus::Dynamic
            };

            let rigid_body = RigidBodyBuilder::new(status).translation(fk * shift, 0.0, fi * shift);
            let collider = ColliderBuilder::ball(rad).density(1.0);
            let child_entity = commands
                .spawn((collider, rigid_body))
                .current_entity()
                .unwrap();

            // Vertical joint.
            if i > 0 {
                let parent_entity = *body_entities.last().unwrap();
                let joint = BallJoint::new(Point3::origin(), Point3::new(0.0, 0.0, -shift));
                commands.spawn((JointBuilderComponent::new(
                    joint,
                    parent_entity,
                    child_entity,
                ),));
            }

            // Horizontal joint.
            if k > 0 {
                let parent_index = body_entities.len() - num;
                let parent_entity = body_entities[parent_index];
                let joint = BallJoint::new(Point3::origin(), Point3::new(-shift, 0.0, 0.0));
                commands.spawn((JointBuilderComponent::new(
                    joint,
                    parent_entity,
                    child_entity,
                ),));
            }

            body_entities.push(child_entity);
        }
    }
}

pub fn setup_physics(mut commands: Commands) {
    create_prismatic_joints(&mut commands, Point3::new(20.0, 10.0, 0.0), 5);
    create_revolute_joints(&mut commands, Point3::new(20.0, 0.0, 0.0), 3);
    create_fixed_joints(&mut commands, Point3::new(0.0, 10.0, 0.0), 5);
    create_ball_joints(&mut commands, 15);
}
