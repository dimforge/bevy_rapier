use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

#[derive(Default)]
pub struct DespawnResource {
    pub entity: Option<Entity>,
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
        .add_plugin(RapierPhysicsPlugin::<NoUserData>::default())
        .add_plugin(RapierDebugRenderPlugin::default())
        .add_startup_system(setup_graphics)
        .add_startup_system(setup_physics)
        .add_system(despawn)
        .run();
}

fn setup_graphics(mut commands: Commands) {
    const HALF_SIZE: f32 = 100.0;

    commands.spawn_bundle(PerspectiveCameraBundle {
        transform: Transform::from_matrix(
            Mat4::look_at_rh(
                Vec3::new(-30.0, 30.0, 100.0),
                Vec3::new(0.0, 10.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
            )
            .inverse(),
        ),
        ..Default::default()
    });
}

pub fn setup_physics(mut commands: Commands, mut despawn: ResMut<DespawnResource>) {
    /*
     * Ground
     */
    let ground_size = 200.1;
    let ground_height = 0.1;

    let ground_entity = commands
        .spawn()
        .insert(Collider::cuboid(ground_size, ground_height, ground_size))
        .insert(Transform::from_xyz(0.0, -ground_height, 0.0))
        .id();
    despawn.entity = Some(ground_entity);
    /*
     * Create the cubes
     */
    let num = 8;
    let rad = 1.0;

    let shift = rad * 2.0 + rad;
    let centerx = shift * (num / 2) as f32;
    let centery = shift / 2.0;
    let centerz = shift * (num / 2) as f32;

    let mut offset = -(num as f32) * (rad * 2.0 + rad) * 0.5;
    let mut color = 0;
    let colors = [
        Color::hsl(220.0, 1.0, 0.3),
        Color::hsl(180.0, 1.0, 0.3),
        Color::hsl(260.0, 1.0, 0.7),
    ];

    for j in 0usize..20 {
        for i in 0..num {
            for k in 0usize..num {
                let x = i as f32 * shift - centerx + offset;
                let y = j as f32 * shift + centery + 3.0;
                let z = k as f32 * shift - centerz + offset;
                color += 1;

                commands
                    .spawn()
                    .insert(RigidBody::Dynamic)
                    .insert(Transform::from_xyz(x, y, z))
                    .insert(Collider::cuboid(rad, rad, rad))
                    .insert(ColliderDebugColor(colors[color % 3]));
            }
        }

        offset -= 0.05 * rad * (num as f32 - 1.0);
    }
}

pub fn despawn(mut commands: Commands, time: Res<Time>, mut despawn: ResMut<DespawnResource>) {
    if time.seconds_since_startup() > 5.0 {
        if let Some(entity) = despawn.entity {
            println!("Despawning ground entity");
            commands.entity(entity).despawn();
            despawn.entity = None;
        }
    }
}
