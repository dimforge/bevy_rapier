extern crate nalgebra as na;
extern crate rapier3d as rapier; // For the debug UI.

use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

use na::{Isometry3, Point3, Unit, Vector3};
use rapier::dynamics::{FixedJoint, PrismaticJoint, RevoluteJoint, RigidBodyType, SphericalJoint};
use rapier3d::pipeline::PhysicsPipeline;
use ui::DebugUiPlugin;

#[path = "../../src_debug_ui/mod.rs"]
mod ui;

fn main() {
    App::new()
        .insert_resource(ClearColor(Color::rgb(
            0xF9 as f32 / 255.0,
            0xF9 as f32 / 255.0,
            0xFF as f32 / 255.0,
        )))
        .insert_resource(Msaa::default())
        .add_plugins(DefaultPlugins)
        .add_plugin(RapierPhysicsPlugin::<NoUserData>::default())
        .add_plugin(RapierRenderPlugin)
        .add_plugin(DebugUiPlugin)
        .add_startup_system(setup_graphics)
        .add_startup_system(setup_physics)
        .add_startup_system(enable_physics_profiling)
        .run();
}

fn enable_physics_profiling(mut pipeline: ResMut<PhysicsPipeline>) {
    pipeline.counters.enable()
}

fn setup_graphics(mut commands: Commands) {
    const HALF_SIZE: f32 = 100.0;

    commands.spawn_bundle(DirectionalLightBundle {
        directional_light: DirectionalLight {
            illuminance: 10000.0,
            // Configure the projection to better fit the scene
            shadow_projection: OrthographicProjection {
                left: -HALF_SIZE,
                right: HALF_SIZE,
                bottom: -HALF_SIZE,
                top: HALF_SIZE,
                near: -10.0 * HALF_SIZE,
                far: 100.0 * HALF_SIZE,
                ..Default::default()
            },
            shadows_enabled: true,
            ..Default::default()
        },
        transform: Transform {
            translation: Vec3::new(10.0, 2.0, 10.0),
            rotation: Quat::from_rotation_x(-std::f32::consts::FRAC_PI_4),
            ..Default::default()
        },
        ..Default::default()
    });
    commands.spawn_bundle(PerspectiveCameraBundle {
        transform: Transform::from_matrix(Mat4::face_toward(
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

    let body = RigidBodyBundle {
        body_type: RigidBodyType::Static.into(),
        position: origin.into(),
        ..Default::default()
    };
    let collider = ColliderBundle {
        shape: ColliderShape::cuboid(rad, rad, rad).into(),
        ..Default::default()
    };

    let mut curr_parent = commands
        .spawn_bundle(body)
        .insert_bundle(collider)
        .insert(ColliderDebugRender::default())
        .insert(ColliderPositionSync::Discrete)
        .id();

    for i in 0..num {
        let z = origin.z + (i + 1) as f32 * shift;

        let rigid_body = RigidBodyBundle {
            position: [origin.x, origin.y, z].into(),
            ..RigidBodyBundle::default()
        };

        let collider = ColliderBundle {
            shape: ColliderShape::cuboid(rad, rad, rad).into(),
            ..ColliderBundle::default()
        };

        let curr_child = commands
            .spawn_bundle(rigid_body)
            .insert_bundle(collider)
            .insert(ColliderDebugRender::with_id(i))
            .insert(ColliderPositionSync::Discrete)
            .id();

        let axis = if i % 2 == 0 {
            Unit::new_normalize(Vector3::new(1.0, 1.0, 0.0))
        } else {
            Unit::new_normalize(Vector3::new(-1.0, 1.0, 0.0))
        };

        let prism = PrismaticJoint::new(axis)
            .local_anchor2(Point3::new(0.0, 0.0, -shift))
            .limit_axis([-2.0, 2.0]);

        commands.spawn_bundle((JointBuilderComponent::new(prism, curr_parent, curr_child),));

        curr_parent = curr_child;
    }
}

fn create_revolute_joints(commands: &mut Commands, origin: Point3<f32>, num: usize) {
    let rad = 0.4;
    let shift = 2.0;

    let ground = RigidBodyBundle {
        body_type: RigidBodyType::Static.into(),
        position: [origin.x, origin.y, 0.0].into(),
        ..RigidBodyBundle::default()
    };

    let collider = ColliderBundle {
        shape: ColliderShape::cuboid(rad, rad, rad).into(),
        ..ColliderBundle::default()
    };

    let mut curr_parent = commands
        .spawn_bundle(ground)
        .insert_bundle(collider)
        .insert(ColliderDebugRender::default())
        .insert(ColliderPositionSync::Discrete)
        .id();

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
            let rigid_body = RigidBodyBundle {
                position: positions[k].into(),
                ..RigidBodyBundle::default()
            };

            let collider = ColliderBundle {
                shape: ColliderShape::cuboid(rad, rad, rad).into(),
                ..ColliderBundle::default()
            };

            handles[k] = commands
                .spawn_bundle(rigid_body)
                .insert_bundle(collider)
                .insert(ColliderDebugRender::with_id(i))
                .insert(ColliderPositionSync::Discrete)
                .id();
        }

        // Setup four joints.
        let x = Vector3::x_axis();
        let z = Vector3::z_axis();

        let revs = [
            RevoluteJoint::new(z).local_anchor2(Point3::new(0.0, 0.0, -shift)),
            RevoluteJoint::new(x).local_anchor2(Point3::new(-shift, 0.0, 0.0)),
            RevoluteJoint::new(z).local_anchor2(Point3::new(0.0, 0.0, -shift)),
            RevoluteJoint::new(x).local_anchor2(Point3::new(shift, 0.0, 0.0)),
        ];

        commands.spawn_bundle((JointBuilderComponent::new(revs[0], curr_parent, handles[0]),));
        commands.spawn_bundle((JointBuilderComponent::new(revs[1], handles[0], handles[1]),));
        commands.spawn_bundle((JointBuilderComponent::new(revs[2], handles[1], handles[2]),));
        commands.spawn_bundle((JointBuilderComponent::new(revs[3], handles[2], handles[3]),));

        curr_parent = handles[3];
    }
}

fn create_fixed_joints(commands: &mut Commands, origin: Point3<f32>, num: usize) {
    let rad = 0.4;
    let shift = 1.0;

    let mut body_entities = Vec::new();
    let mut color = 0;

    for k in 0..num {
        for i in 0..num {
            let fk = k as f32;
            let fi = i as f32;
            color += 1;

            // NOTE: the num - 2 test is to avoid two consecutive
            // fixed bodies. Because physx will crash if we add
            // a joint between these.
            let body_type = if i == 0 && (k % 4 == 0 && k != num - 2 || k == num - 1) {
                RigidBodyType::Static
            } else {
                RigidBodyType::Dynamic
            };

            let rigid_body = RigidBodyBundle {
                body_type: body_type.into(),
                position: [origin.x + fk * shift, origin.y, origin.z + fi * shift].into(),
                ..RigidBodyBundle::default()
            };

            let collider = ColliderBundle {
                shape: ColliderShape::ball(rad).into(),
                ..ColliderBundle::default()
            };

            let child_entity = commands
                .spawn_bundle(rigid_body)
                .insert_bundle(collider)
                .insert(ColliderDebugRender::with_id(color))
                .insert(ColliderPositionSync::Discrete)
                .id();

            // Vertical joint.
            if i > 0 {
                let parent_entity = *body_entities.last().unwrap();
                let joint = FixedJoint::new().local_anchor2(point![0.0, 0.0, -shift]);
                commands.spawn_bundle((JointBuilderComponent::new(
                    joint,
                    parent_entity,
                    child_entity,
                ),));
            }

            // Horizontal joint.
            if k > 0 {
                let parent_index = body_entities.len() - num;
                let parent_entity = body_entities[parent_index];
                let joint = FixedJoint::new().local_anchor2(point![-shift, 0.0, 0.0]);
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

fn create_ball_joints(commands: &mut Commands, num: usize) {
    let rad = 0.4;
    let shift = 1.0;

    let mut body_entities = Vec::new();
    let mut color = 0;

    for k in 0..num {
        for i in 0..num {
            let fk = k as f32;
            let fi = i as f32;
            color += 1;

            let body_type = if i == 0 && (k % 4 == 0 || k == num - 1) {
                RigidBodyType::Static
            } else {
                RigidBodyType::Dynamic
            };

            let rigid_body = RigidBodyBundle {
                body_type: body_type.into(),
                position: [fk * shift, 0.0, fi * shift].into(),
                ..Default::default()
            };

            let collider = ColliderBundle {
                shape: ColliderShape::ball(rad).into(),
                ..Default::default()
            };

            let child_entity = commands
                .spawn_bundle(collider)
                .insert_bundle(rigid_body)
                .insert(ColliderDebugRender::with_id(color))
                .insert(ColliderPositionSync::Discrete)
                .id();

            // Vertical joint.
            if i > 0 {
                let parent_entity = *body_entities.last().unwrap();
                let joint = SphericalJoint::new().local_anchor2(Point3::new(0.0, 0.0, -shift));
                commands.spawn_bundle((JointBuilderComponent::new(
                    joint,
                    parent_entity,
                    child_entity,
                ),));
            }

            // Horizontal joint.
            if k > 0 {
                let parent_index = body_entities.len() - num;
                let parent_entity = body_entities[parent_index];
                let joint = SphericalJoint::new().local_anchor2(Point3::new(-shift, 0.0, 0.0));
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

pub fn setup_physics(mut commands: Commands) {
    create_prismatic_joints(&mut commands, Point3::new(20.0, 10.0, 0.0), 5);
    create_revolute_joints(&mut commands, Point3::new(20.0, 0.0, 0.0), 3);
    create_fixed_joints(&mut commands, Point3::new(0.0, 10.0, 0.0), 5);
    create_ball_joints(&mut commands, 15);
}
