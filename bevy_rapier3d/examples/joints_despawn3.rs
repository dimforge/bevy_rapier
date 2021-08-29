extern crate nalgebra as na;
extern crate rapier3d as rapier; // For the debug UI.

use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

use bevy::render::pass::ClearColor;
use na::{Isometry3, Point3, Unit, Vector3};
use rapier::dynamics::{BallJoint, FixedJoint, PrismaticJoint, RevoluteJoint, RigidBodyType};
use rapier3d::pipeline::PhysicsPipeline;
use ui::DebugUiPlugin;

#[path = "../../src_debug_ui/mod.rs"]
mod ui;

#[derive(Default)]
pub struct DespawnResource {
    entities: Vec<Entity>,
}

fn main() {
    App::new()
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
    commands.spawn_bundle(PointLightBundle {
        transform: Transform::from_translation(Vec3::new(100.0, 10.0, 200.0)),
        point_light: PointLight {
            intensity: 100_000.0,
            range: 3000.0,
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

fn create_prismatic_joints(
    commands: &mut Commands,
    origin: Point3<f32>,
    num: usize,
    despawn: &mut ResMut<DespawnResource>,
) {
    let rad = 0.4;
    let shift = 1.0;

    let body = RigidBodyBundle {
        body_type: RigidBodyType::Static,
        position: origin.into(),
        ..Default::default()
    };
    let collider = ColliderBundle {
        shape: ColliderShape::cuboid(rad, rad, rad),
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
            shape: ColliderShape::cuboid(rad, rad, rad),
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

        let entity = commands
            .spawn_bundle((JointBuilderComponent::new(prism, curr_parent, curr_child),))
            .id();

        if i == 2 {
            despawn.entities.push(entity);
        }

        curr_parent = curr_child;
    }
}

fn create_revolute_joints(
    commands: &mut Commands,
    origin: Point3<f32>,
    num: usize,
    despawn: &mut ResMut<DespawnResource>,
) {
    let rad = 0.4;
    let shift = 2.0;

    let ground = RigidBodyBundle {
        body_type: RigidBodyType::Static,
        position: [origin.x, origin.y, 0.0].into(),
        ..RigidBodyBundle::default()
    };

    let collider = ColliderBundle {
        shape: ColliderShape::cuboid(rad, rad, rad),
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
                shape: ColliderShape::cuboid(rad, rad, rad),
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
        let o = Point3::origin();
        let x = Vector3::x_axis();
        let z = Vector3::z_axis();

        let revs = [
            RevoluteJoint::new(o, z, Point3::new(0.0, 0.0, -shift), z),
            RevoluteJoint::new(o, x, Point3::new(-shift, 0.0, 0.0), x),
            RevoluteJoint::new(o, z, Point3::new(0.0, 0.0, -shift), z),
            RevoluteJoint::new(o, x, Point3::new(shift, 0.0, 0.0), x),
        ];

        let entity1 = commands
            .spawn_bundle((JointBuilderComponent::new(revs[0], curr_parent, handles[0]),))
            .id();
        let entity2 = commands
            .spawn_bundle((JointBuilderComponent::new(revs[1], handles[0], handles[1]),))
            .id();
        let entity3 = commands
            .spawn_bundle((JointBuilderComponent::new(revs[2], handles[1], handles[2]),))
            .id();
        let entity4 = commands
            .spawn_bundle((JointBuilderComponent::new(revs[3], handles[2], handles[3]),))
            .id();

        if i == 1 {
            despawn.entities.push(entity1);
            despawn.entities.push(entity2);
            despawn.entities.push(entity3);
            despawn.entities.push(entity4);
        }

        curr_parent = handles[3];
    }
}

fn create_fixed_joints(
    commands: &mut Commands,
    origin: Point3<f32>,
    num: usize,
    despawn: &mut ResMut<DespawnResource>,
) {
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
                body_type,
                position: [origin.x + fk * shift, origin.y, origin.z + fi * shift].into(),
                ..RigidBodyBundle::default()
            };

            let collider = ColliderBundle {
                shape: ColliderShape::ball(rad),
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
                let joint = FixedJoint::new(
                    Isometry3::identity(),
                    Isometry3::translation(0.0, 0.0, -shift),
                );
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
                let joint = FixedJoint::new(
                    Isometry3::identity(),
                    Isometry3::translation(-shift, 0.0, 0.0),
                );
                let entity = commands
                    .spawn()
                    .insert_bundle((JointBuilderComponent::new(
                        joint,
                        parent_entity,
                        child_entity,
                    ),))
                    .id();
                if k == 2 {
                    despawn.entities.push(entity);
                }
            }

            body_entities.push(child_entity);
        }
    }
}

fn create_ball_joints(commands: &mut Commands, num: usize, despawn: &mut ResMut<DespawnResource>) {
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
                body_type,
                position: [fk * shift, 0.0, fi * shift].into(),
                ..Default::default()
            };

            let collider = ColliderBundle {
                shape: ColliderShape::ball(rad),
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
                let joint = BallJoint::new(Point3::origin(), Point3::new(0.0, 0.0, -shift));
                let entity = commands
                    .spawn()
                    .insert_bundle((JointBuilderComponent::new(
                        joint,
                        parent_entity,
                        child_entity,
                    ),))
                    .id();

                if i == 2 {
                    despawn.entities.push(entity);
                }
            }

            // Horizontal joint.
            if k > 0 {
                let parent_index = body_entities.len() - num;
                let parent_entity = body_entities[parent_index];
                let joint = BallJoint::new(Point3::origin(), Point3::new(-shift, 0.0, 0.0));
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

pub fn setup_physics(mut commands: Commands, mut despawn: ResMut<DespawnResource>) {
    create_prismatic_joints(&mut commands, Point3::new(20.0, 10.0, 0.0), 5, &mut despawn);
    create_revolute_joints(&mut commands, Point3::new(20.0, 0.0, 0.0), 3, &mut despawn);
    create_fixed_joints(&mut commands, Point3::new(0.0, 10.0, 0.0), 5, &mut despawn);
    create_ball_joints(&mut commands, 15, &mut despawn);
}

pub fn despawn(mut commands: Commands, time: Res<Time>, mut despawn: ResMut<DespawnResource>) {
    if time.seconds_since_startup() > 10.0 {
        for entity in &despawn.entities {
            println!("Despawning joint entity");
            commands.entity(*entity).despawn();
        }
        despawn.entities.clear();
    }
}
