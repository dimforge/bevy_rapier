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
                Vec3::new(10.0, 3.0, 0.0),
                Vec3::new(0.0, 3.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
            )
            .inverse(),
        ),
        ..Default::default()
    });
}

pub fn setup_physics(mut commands: Commands) {
    /*
     * The ground
     */
    let ground_size = 5.0;
    let ground_height = 0.1;

    commands
        .spawn_bundle(TransformBundle::from(Transform::from_xyz(0.0, -ground_height, 0.0)))
        .insert(Collider::cuboid(ground_size, ground_height, ground_size));

    /*
     * A rectangle that only rotates along the `x` axis.
     */
    commands
        .spawn_bundle(TransformBundle::from(Transform::from_xyz(0.0, 3.0, 0.0)))
        .insert(RigidBody::Dynamic)
        .insert(
            LockedAxes::TRANSLATION_LOCKED
                | LockedAxes::ROTATION_LOCKED_Y
                | LockedAxes::ROTATION_LOCKED_Z,
        )
        .insert(Collider::cuboid(0.2, 0.6, 2.0));

    /*
     * A tilted cuboid that cannot rotate.
     */
    commands
        .spawn_bundle(TransformBundle::from(Transform::from_xyz(0.0, 5.0, 0.0).with_rotation(Quat::from_rotation_x(1.0)))
        .insert(RigidBody::Dynamic)
        .insert(LockedAxes::ROTATION_LOCKED)
        .insert(Collider::cuboid(0.6, 0.4, 0.4));
}
