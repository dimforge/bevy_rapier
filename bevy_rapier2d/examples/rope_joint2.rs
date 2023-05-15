use bevy::prelude::*;
use bevy_rapier2d::prelude::*;

fn main() {
    App::new()
        .insert_resource(ClearColor(Color::rgb(
            0xF9 as f32 / 255.0,
            0xF9 as f32 / 255.0,
            0xFF as f32 / 255.0,
        )))
        .add_plugins(DefaultPlugins)
        .add_plugin(RapierPhysicsPlugin::<NoUserData>::pixels_per_meter(100.0))
        .add_plugin(RapierDebugRenderPlugin::default())
        .add_startup_system(setup_graphics)
        .add_startup_system(setup_physics)
        .run();
}

fn setup_graphics(mut commands: Commands) {
    commands.spawn(Camera2dBundle {
        transform: Transform::from_xyz(0.0, -200.0, 0.0),
        ..default()
    });
}

pub fn setup_physics(mut commands: Commands) {
    let rad = 10.0;
    let rope_length = rad * 10.0;

    let parent = commands
        .spawn((
            TransformBundle::from(Transform::from_xyz(0.0, 0.0, 0.0)),
            RigidBody::Fixed,
            Collider::cuboid(rad, rad),
        ))
        .id();

    let joint = RopeJointBuilder::new()
        .local_anchor2(Vec2::new(0.0, rope_length))
        .limits([0.0, rope_length]);

    commands
        .spawn((
            TransformBundle::from(Transform::from_xyz(0.0, -rad * 2.0, 0.0)),
            RigidBody::Dynamic,
            Collider::cuboid(rad, rad),
        ))
        .insert(ImpulseJoint::new(parent, joint));
}
