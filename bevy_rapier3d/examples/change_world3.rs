use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

const N_WORLDS: WorldId = 5;
const WORLD_CHANGE_DELAY_SEC: f32 = 3.0;

#[derive(Component)]
/// Denotes which object(s) to change the world of
struct ChangeWorld;

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
        .add_systems(Startup, (setup_graphics, setup_physics))
        .add_systems(Update, change_world)
        .run();
}

fn change_world(mut query: Query<&mut PhysicsWorld, With<ChangeWorld>>, time: Res<Time>) {
    for mut bw in query.iter_mut() {
        if time.elapsed_seconds() > (bw.world_id as f32 + 1.0) * WORLD_CHANGE_DELAY_SEC {
            let new_world_id = bw.world_id + 1;

            if new_world_id != N_WORLDS {
                println!("Changing world to {new_world_id}.");
                bw.world_id = new_world_id;
            }
        }
    }
}

fn setup_graphics(mut commands: Commands) {
    commands.spawn(Camera3dBundle {
        transform: Transform::from_xyz(0.0, 3.0, -10.0)
            .looking_at(Vec3::new(0.0, 0.0, 0.0), Vec3::Y),
        ..Default::default()
    });
}

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
            RigidBody::Fixed,
            PhysicsWorld { world_id },
        ));
    }

    /*
     * Create the cube
     */
    commands.spawn((
        TransformBundle::from(Transform::from_xyz(0.0, 3.0, 0.0)),
        RigidBody::Dynamic,
        PhysicsWorld {
            world_id: DEFAULT_WORLD_ID,
        },
        ChangeWorld,
    )).with_children(|p| {
        p.spawn((
            TransformBundle::from_transform(Transform::from_xyz(0.0, 0.0, 0.0)),
            Collider::cuboid(0.5, 0.5, 0.5),
            ColliderDebugColor(Color::hsl(260.0, 1.0, 0.7)),
        ));
    });
}
