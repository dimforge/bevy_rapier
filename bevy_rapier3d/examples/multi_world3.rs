use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

const N_WORLDS: usize = 5;

fn main() {
    App::new()
        .insert_resource(ClearColor(Color::rgb(
            0xF9 as f32 / 255.0,
            0xF9 as f32 / 255.0,
            0xFF as f32 / 255.0,
        )))
        .add_plugins((
            DefaultPlugins,
            RapierPhysicsPlugin::<NoUserData>::default(),
            RapierDebugRenderPlugin::default(),
        ))
        .add_systems(Startup, (setup_physics, setup_graphics))
        .add_systems(Update, move_middle_world)
        // .add_systems(Update, change_world)
        // .add_systems(Update, despawn_last)
        .run();
}

fn setup_graphics(mut commands: Commands) {
    commands.spawn(Camera3dBundle {
        transform: Transform::from_xyz(0.0, 3.0, -10.0)
            .looking_at(Vec3::new(0.0, 0.0, 0.0), Vec3::Y),
        ..Default::default()
    });
}

#[derive(Component)]
struct Platform {
    starting_y: f32,
}

fn move_middle_world(
    time: Res<Time>,
    mut query: Query<(&mut Transform, &PhysicsWorld, &Platform)>,
) {
    for (mut transform, world, platform) in query.iter_mut() {
        if world.world_id == N_WORLDS / 2 {
            transform.translation.y = platform.starting_y + -time.elapsed_seconds().sin();
        }
    }
}

/// Demonstrates despawning an entity removing it from its world
// fn despawn_last(query: Query<(&PhysicsWorld, Entity)>, mut commands: Commands) {
//     for (bw, entity) in query.iter() {
//         if bw.world_id == N_WORLDS - 1 {
//             commands.entity(entity).despawn_recursive();
//         }
//     }
// }

/// Demonstrates how easy it is to move one entity to another world.
// fn change_world(mut query: Query<&mut PhysicsWorld>) {
//     for mut bw in query.iter_mut() {
//         if bw.world_id == 1 {
//             bw.world_id = 0;
//         }
//     }
// }

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

        let starting_y = (world_id as f32) * -0.5 - ground_height;

        commands.spawn((
            TransformBundle::from(Transform::from_xyz(0.0, starting_y, 0.0)),
            Collider::cuboid(ground_size, ground_height, ground_size),
            ColliderDebugColor(color),
            Platform { starting_y },
            PhysicsWorld { world_id },
        ));

        /*
         * Create the cube
         */

        commands.spawn((
            TransformBundle::from(Transform::from_xyz(0.0, 1.0 + world_id as f32 * 5.0, 0.0)),
            RigidBody::Dynamic,
            Collider::cuboid(0.5, 0.5, 0.5),
            ColliderDebugColor(color),
            PhysicsWorld { world_id },
        ));
    }
}
