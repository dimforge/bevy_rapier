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
        translation: Vec3::new(0.0, 200.0, 0.0),
        ..Transform::identity()
    };
    commands.spawn_bundle(camera);
}

pub fn setup_physics(mut commands: Commands) {
    /*
     * The ground
     */
    let ground_size = 500.0;
    let ground_height = 10.0;

    commands
        .spawn()
        .insert(Collider::cuboid(ground_size, ground_height))
        .insert(Transform::from_xyz(0.0, -ground_height, 0.0));

    /*
     * A rectangle that only rotate.
     */
    commands
        .spawn()
        .insert(RigidBody::Dynamic)
        .insert(LockedAxes::TRANSLATION_LOCKED)
        .insert(Collider::cuboid(200.0, 60.0))
        .insert(Transform::from_xyz(0.0, 300.0, 0.0));

    /*
     * A tilted cuboid that cannot rotate.
     */
    commands
        .spawn()
        .insert(RigidBody::Dynamic)
        .insert(LockedAxes::ROTATION_LOCKED)
        .insert(Collider::cuboid(60.0, 40.0))
        .insert(Transform::from_xyz(50.0, 500.0, 0.0).with_rotation(Quat::from_rotation_z(1.0)));
}
