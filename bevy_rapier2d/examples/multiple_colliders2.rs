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
    commands.spawn_bundle(Camera2dBundle::default());
}

pub fn setup_physics(mut commands: Commands) {
    /*
     * Ground
     */
    let ground_size = 500.0;
    let ground_height = 1.0;

    commands
        .spawn_bundle(TransformBundle::from(Transform::from_xyz(
            0.0,
            -ground_height,
            0.0,
        )))
        .insert(Collider::cuboid(ground_size, ground_height));

    /*
     * Create the cubes
     */
    let num = 4;
    let rad = 2.0;

    let shift = rad * 4.0 + rad;
    let centerx = shift * (num / 2) as f32;
    let centery = shift / 2.0;

    let mut offset = -(num as f32) * (rad * 2.0 + rad) * 0.5;

    for j in 0usize..20 {
        for i in 0..num {
            let x = i as f32 * shift * 5.0 - centerx + offset;
            let y = j as f32 * (shift * 5.0) + centery + 3.0;

            commands
                .spawn_bundle(TransformBundle::from(Transform::from_xyz(x, y, 0.0)))
                .insert(RigidBody::Dynamic)
                .with_children(|children| {
                    children.spawn().insert(Collider::cuboid(rad * 10.0, rad));
                    children
                        .spawn_bundle(TransformBundle::from(Transform::from_xyz(
                            rad * 10.0,
                            rad * 10.0,
                            0.0,
                        )))
                        .insert(Collider::cuboid(rad, rad * 10.0));
                    children
                        .spawn_bundle(TransformBundle::from(Transform::from_xyz(
                            -rad * 10.0,
                            rad * 10.0,
                            0.0,
                        )))
                        .insert(Collider::cuboid(rad, rad * 10.0));
                });
        }

        offset -= 0.05 * rad * (num as f32 - 1.0);
    }
}
