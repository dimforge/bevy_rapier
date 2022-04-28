use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

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
        .add_plugin(RapierDebugRenderPlugin::default())
        .add_startup_system(setup_graphics)
        .add_startup_system(setup_physics)
        .run();
}

fn setup_graphics(mut commands: Commands) {
    commands.spawn_bundle(PerspectiveCameraBundle {
        transform: Transform::from_matrix(
            Mat4::look_at_rh(
                Vec3::new(15.0, 5.0, 42.0),
                Vec3::new(13.0, 1.0, 1.0),
                Vec3::new(0.0, 1.0, 0.0),
            )
            .inverse(),
        ),
        ..Default::default()
    });
}

fn create_prismatic_joints(commands: &mut Commands, origin: Vect, num: usize) {
    let rad = 0.4;
    let shift = 1.0;

    let mut curr_parent = commands
        .spawn()
        .insert(RigidBody::Fixed)
        .insert(Transform::from_xyz(origin.x, origin.y, origin.z))
        .insert(Collider::cuboid(rad, rad, rad))
        .id();

    for i in 0..num {
        let dz = (i + 1) as f32 * shift;

        let axis = if i % 2 == 0 {
            Vec3::new(1.0, 1.0, 0.0)
        } else {
            Vec3::new(-1.0, 1.0, 0.0)
        };

        let prism = PrismaticJointBuilder::new(axis)
            .local_anchor2(Vec3::new(0.0, 0.0, -shift))
            .limits([-2.0, 2.0]);
        let joint = ImpulseJoint::new(curr_parent, prism);

        curr_parent = commands
            .spawn()
            .insert(RigidBody::Dynamic)
            .insert(Transform::from_xyz(origin.x, origin.y, origin.z + dz))
            .insert(Collider::cuboid(rad, rad, rad))
            .insert(joint)
            .id();
    }
}

fn create_revolute_joints(commands: &mut Commands, origin: Vec3, num: usize) {
    let rad = 0.4;
    let shift = 2.0;

    let mut curr_parent = commands
        .spawn()
        .insert(RigidBody::Fixed)
        .insert(Transform::from_xyz(origin.x, origin.y, 0.0))
        .insert(Collider::cuboid(rad, rad, rad))
        .id();

    for i in 0..num {
        // Create four bodies.
        let z = origin.z + i as f32 * shift * 2.0 + shift;
        let positions = [
            Vec3::new(origin.x, origin.y, z),
            Vec3::new(origin.x + shift, origin.y, z),
            Vec3::new(origin.x + shift, origin.y, z + shift),
            Vec3::new(origin.x, origin.y, z + shift),
        ];

        let mut handles = [curr_parent; 4];
        for k in 0..4 {
            handles[k] = commands
                .spawn()
                .insert(RigidBody::Dynamic)
                .insert(Transform::from_translation(positions[k]))
                .insert(Collider::cuboid(rad, rad, rad))
                .id();
        }

        // Setup four joints.
        let x = Vec3::X;
        let z = Vec3::Z;

        let revs = [
            RevoluteJointBuilder::new(z).local_anchor2(Vec3::new(0.0, 0.0, -shift)),
            RevoluteJointBuilder::new(x).local_anchor2(Vec3::new(-shift, 0.0, 0.0)),
            RevoluteJointBuilder::new(z).local_anchor2(Vec3::new(0.0, 0.0, -shift)),
            RevoluteJointBuilder::new(x).local_anchor2(Vec3::new(shift, 0.0, 0.0)),
        ];

        commands
            .entity(handles[0])
            .insert(ImpulseJoint::new(curr_parent, revs[0]));
        commands
            .entity(handles[1])
            .insert(ImpulseJoint::new(handles[0], revs[1]));
        commands
            .entity(handles[2])
            .insert(ImpulseJoint::new(handles[1], revs[2]));
        commands
            .entity(handles[3])
            .insert(ImpulseJoint::new(handles[2], revs[3]));

        curr_parent = handles[3];
    }
}

fn create_fixed_joints(commands: &mut Commands, origin: Vec3, num: usize) {
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
            let rigid_body = if i == 0 && (k % 4 == 0 && k != num - 2 || k == num - 1) {
                RigidBody::Fixed
            } else {
                RigidBody::Dynamic
            };

            let transform =
                Transform::from_xyz(origin.x + fk * shift, origin.y, origin.z + fi * shift);
            let child_entity = commands
                .spawn()
                .insert(rigid_body)
                .insert(transform)
                .insert(Collider::ball(rad))
                .id();

            // Vertical joint.
            if i > 0 {
                let parent_entity = *body_entities.last().unwrap();
                let joint = FixedJointBuilder::new().local_anchor2(Vec3::new(0.0, 0.0, -shift));
                commands.entity(child_entity).with_children(|children| {
                    // NOTE: we want to attach multiple impulse joints to this entity, so
                    //       we need to add the components to children of the entity. Otherwise
                    //       the second joint component would just overwrite the first one.
                    children
                        .spawn()
                        .insert(ImpulseJoint::new(parent_entity, joint));
                });
            }

            // Horizontal joint.
            if k > 0 {
                let parent_index = body_entities.len() - num;
                let parent_entity = body_entities[parent_index];
                let joint = FixedJointBuilder::new().local_anchor2(Vec3::new(-shift, 0.0, 0.0));
                commands.entity(child_entity).with_children(|children| {
                    // NOTE: we want to attach multiple impulse joints to this entity, so
                    //       we need to add the components to children of the entity. Otherwise
                    //       the second joint component would just overwrite the first one.
                    children
                        .spawn()
                        .insert(ImpulseJoint::new(parent_entity, joint));
                });
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

            let rigid_body = if i == 0 && (k % 4 == 0 || k == num - 1) {
                RigidBody::Fixed
            } else {
                RigidBody::Dynamic
            };

            let transform = Transform::from_xyz(fk * shift, 0.0, fi * shift);
            let child_entity = commands
                .spawn()
                .insert(rigid_body)
                .insert(Collider::ball(rad))
                .insert(transform)
                .id();

            // Vertical joint.
            if i > 0 {
                let parent_entity = *body_entities.last().unwrap();
                let joint = SphericalJointBuilder::new().local_anchor2(Vec3::new(0.0, 0.0, -shift));
                commands.entity(child_entity).with_children(|children| {
                    // NOTE: we want to attach multiple impulse joints to this entity, so
                    //       we need to add the components to children of the entity. Otherwise
                    //       the second joint component would just overwrite the first one.
                    children
                        .spawn()
                        .insert(ImpulseJoint::new(parent_entity, joint));
                });
            }

            // Horizontal joint.
            if k > 0 {
                let parent_index = body_entities.len() - num;
                let parent_entity = body_entities[parent_index];
                let joint = SphericalJointBuilder::new().local_anchor2(Vec3::new(-shift, 0.0, 0.0));
                commands.entity(child_entity).with_children(|children| {
                    // NOTE: we want to attach multiple impulse joints to this entity, so
                    //       we need to add the components to children of the entity. Otherwise
                    //       the second joint component would just overwrite the first one.
                    children
                        .spawn()
                        .insert(ImpulseJoint::new(parent_entity, joint));
                });
            }

            body_entities.push(child_entity);
        }
    }
}

pub fn setup_physics(mut commands: Commands) {
    create_prismatic_joints(&mut commands, Vec3::new(20.0, 10.0, 0.0), 5);
    create_revolute_joints(&mut commands, Vec3::new(20.0, 0.0, 0.0), 3);
    create_fixed_joints(&mut commands, Vec3::new(0.0, 10.0, 0.0), 5);
    create_ball_joints(&mut commands, 15);
}
