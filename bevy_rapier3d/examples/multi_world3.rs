use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

fn main() {
    App::new()
        .insert_resource(ClearColor(Color::rgb(
            0xF9 as f32 / 255.0,
            0xF9 as f32 / 255.0,
            0xFF as f32 / 255.0,
        )))
        .add_plugins(DefaultPlugins)
        .add_plugin(RapierPhysicsPlugin::<NoUserData>::default())
        .add_plugin(RapierDebugRenderPlugin::default())
        .add_startup_system(setup_graphics)
        .add_startup_system(setup_physics)
        .add_system(move_middle_world)
        .run();
}

fn setup_graphics(mut commands: Commands) {
    commands.spawn(Camera3dBundle {
        transform: Transform::from_xyz(0.0, 3.0, -10.0)
            .looking_at(Vec3::new(0.0, 0.0, 0.0), Vec3::Y),
        ..Default::default()
    });
}

fn move_middle_world(time: Res<Time>, mut query: Query<(&mut Transform, &BodyWorld)>) {
    for (mut transform, world) in query.iter_mut() {
        if world.world_id == N_WORLDS / 2 {
            transform.translation.y = -time.elapsed_seconds().sin();
        }
    }
}

const N_WORLDS: usize = 6;

pub fn setup_physics(mut context: ResMut<RapierContext>, mut commands: Commands) {
    for _ in 1..N_WORLDS {
        context.add_world(RapierWorld::default());
    }

    for world_id in 0..N_WORLDS {
        let color = [
            Color::hsl(220.0, 1.0, 0.3),
            Color::hsl(180.0, 1.0, 0.3),
            Color::hsl(260.0, 1.0, 0.7),
        ][world_id % 3];

        /*
         * Ground
         */
        let ground_size = 5.1;
        let ground_height = 0.1;

        commands.spawn((
            TransformBundle::from(Transform::from_xyz(
                0.0,
                (world_id as f32) * -0.5 - ground_height,
                0.0,
            )),
            Collider::cuboid(ground_size, ground_height, ground_size),
            ColliderDebugColor(color),
            BodyWorld { world_id },
        ));

        /*
         * Create the cube
         */

        commands
            .spawn(TransformBundle::from(Transform::from_rotation(
                Quat::from_rotation_x(0.2),
            )))
            .with_children(|child| {
                child.spawn((
                    TransformBundle::from(Transform::from_xyz(
                        0.0,
                        1.0 + world_id as f32 * 5.0,
                        0.0,
                    )),
                    RigidBody::Dynamic,
                    Collider::cuboid(0.5, 0.5, 0.5),
                    ColliderDebugColor(color),
                    BodyWorld { world_id },
                ));
            });
    }
}
