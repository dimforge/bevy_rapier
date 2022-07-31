use bevy::prelude::*;
use bevy_rapier2d::prelude::*;

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
        .add_plugin(RapierPhysicsPlugin::<NoUserData>::pixels_per_meter(100.0))
        .add_plugin(RapierDebugRenderPlugin::default())
        .add_startup_system(setup_graphics)
        .add_startup_system(setup_physics)
        .add_system(despawn)
        .run();
}

fn setup_graphics(mut commands: Commands) {
    commands.spawn_bundle(Camera2dBundle {
        transform: Transform::from_xyz(0.0, -200.0, 0.0),
        ..default()
    });
}

pub fn setup_physics(mut commands: Commands, mut despawn: ResMut<DespawnResource>) {
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
                .spawn_bundle(TransformBundle::from(Transform::from_xyz(
                    fk * shift,
                    -fi * shift,
                    0.0,
                )))
                .insert(rigid_body)
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
                    let entity = cmd
                        .spawn()
                        .insert(ImpulseJoint::new(parent_entity, joint))
                        .id();
                    if i == (numi / 2) || (k % 4 == 0 || k == numk - 1) {
                        despawn.entities.push(entity);
                    }
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
                    let entity = cmd
                        .spawn()
                        .insert(ImpulseJoint::new(parent_entity, joint))
                        .id();
                    if i == (numi / 2) || (k % 4 == 0 || k == numk - 1) {
                        despawn.entities.push(entity);
                    }
                });
            }

            body_entities.push(child_entity);
        }
    }
}

pub fn despawn(mut commands: Commands, time: Res<Time>, mut despawn: ResMut<DespawnResource>) {
    if time.seconds_since_startup() > 4.0 {
        for entity in &despawn.entities {
            println!("Despawning joint entity");
            commands.entity(*entity).despawn();
        }
        despawn.entities.clear();
    }
}
