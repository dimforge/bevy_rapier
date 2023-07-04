use bevy::prelude::*;
use bevy_rapier2d::prelude::*;

fn main() {
    App::new()
        .insert_resource(ClearColor(Color::rgb(
            0xF9 as f32 / 255.0,
            0xF9 as f32 / 255.0,
            0xFF as f32 / 255.0,
        )))
        .add_plugins((
            DefaultPlugins,
            RapierPhysicsPlugin::<NoUserData>::pixels_per_meter(100.0),
            RapierDebugRenderPlugin::default(),
        ))
        .add_systems(Startup, (setup_graphics, setup_physics))
        .run();
}

fn setup_graphics(mut commands: Commands) {
    commands.spawn(Camera2dBundle {
        transform: Transform::from_xyz(0.0, 200.0, 0.0),
        ..default()
    });
}

pub fn setup_physics(mut commands: Commands) {
    /*
     * The ground
     */
    let ground_size = 500.0;
    let ground_height = 10.0;

    commands.spawn((
        TransformBundle::from(Transform::from_xyz(0.0, -ground_height, 0.0)),
        Collider::cuboid(ground_size, ground_height),
    ));

    /*
     * A rectangle that only rotate.
     */
    commands.spawn((
        TransformBundle::from(Transform::from_xyz(0.0, 300.0, 0.0)),
        RigidBody::Dynamic,
        LockedAxes::TRANSLATION_LOCKED,
        Collider::cuboid(200.0, 60.0),
    ));

    /*
     * A tilted cuboid that cannot rotate.
     */
    commands.spawn((
        TransformBundle::from(
            Transform::from_xyz(50.0, 500.0, 0.0).with_rotation(Quat::from_rotation_z(1.0)),
        ),
        RigidBody::Dynamic,
        LockedAxes::ROTATION_LOCKED,
        Collider::cuboid(60.0, 40.0),
    ));
}
