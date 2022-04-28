use bevy::prelude::*;
use bevy_rapier2d::prelude::*;

fn main() {
    App::new()
        .insert_resource(ClearColor(Color::rgb(
            0xF9 as f32 / 255.0,
            0xF9 as f32 / 255.0,
            0xFF as f32 / 255.0,
        )))
        .insert_resource(Msaa::default())
        .add_plugins(DefaultPlugins)
        .add_plugin(RapierPhysicsPlugin::<NoUserData>::pixels_per_meter(100.0))
        .add_plugin(RapierDebugRenderPlugin::default())
        .add_startup_system(setup_graphics)
        .add_startup_system(setup_physics)
        .run();
}

fn setup_graphics(mut commands: Commands) {
    let mut camera = OrthographicCameraBundle::new_2d();
    camera.transform = Transform {
        translation: Vec3::new(0.0, -200.0, 0.0),
        ..Transform::identity()
    };
    commands.spawn_bundle(camera);
}

pub fn setup_physics(mut commands: Commands) {
    // Build the rigid body.
    let rad = 4.0;
    let numi = 40; // Num vertical nodes.
    let numk = 40; // Num horizontal nodes.
    let shift = 10.0;

    let mut body_entities = Vec::new();

    for k in 0..numk {
        for i in 0..numi {
            let fk = k as f32;
            let fi = i as f32;

            let rigid_body = if i == 0 && (k % 4 == 0 || k == numk - 1) {
                RigidBody::Fixed
            } else {
                RigidBody::Dynamic
            };

            let child_entity = commands
                .spawn()
                .insert(rigid_body)
                .insert(Transform::from_xyz(fk * shift, -fi * shift, 0.0))
                .insert(Collider::cuboid(rad, rad))
                .id();

            // Vertical joint.
            if i > 0 {
                let parent_entity = *body_entities.last().unwrap();
                let joint = RevoluteJointBuilder::new().local_anchor2(Vec2::new(0.0, shift));
                commands.entity(child_entity).with_children(|cmd| {
                    // NOTE: we want to attach multiple impulse joints to this entity, so
                    //       we need to add the components to children of the entity. Otherwise
                    //       the second joint component would just overwrite the first one.
                    cmd.spawn().insert(ImpulseJoint::new(parent_entity, joint));
                });
            }

            // Horizontal joint.
            if k > 0 {
                let parent_index = body_entities.len() - numi;
                let parent_entity = body_entities[parent_index];
                let joint = RevoluteJointBuilder::new().local_anchor2(Vec2::new(-shift, 0.0));
                commands.entity(child_entity).with_children(|cmd| {
                    // NOTE: we want to attach multiple impulse joints to this entity, so
                    //       we need to add the components to children of the entity. Otherwise
                    //       the second joint component would just overwrite the first one.
                    cmd.spawn().insert(ImpulseJoint::new(parent_entity, joint));
                });
            }

            body_entities.push(child_entity);
        }
    }
}
