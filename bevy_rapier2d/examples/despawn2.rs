use bevy::prelude::*;
use bevy_rapier2d::prelude::*;

#[derive(Default)]
pub struct DespawnResource {
    pub entities: Vec<Entity>,
}

#[derive(Default)]
pub struct ResizeResource {
    pub entities: Vec<Entity>,
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
        .insert_resource(ResizeResource::default())
        .add_plugins(DefaultPlugins)
        .add_plugin(RapierPhysicsPlugin::<NoUserData>::pixels_per_meter(100.0))
        .add_plugin(RapierDebugRenderPlugin::default())
        .add_startup_system(setup_graphics)
        .add_startup_system(setup_physics)
        .add_system(despawn)
        .add_system(resize)
        .run();
}

fn setup_graphics(mut commands: Commands) {
    commands.spawn_bundle(OrthographicCameraBundle {
        transform: Transform::from_xyz(0.0, 20.0, 0.0),
        ..OrthographicCameraBundle::new_2d()
    });
}

pub fn setup_physics(
    mut commands: Commands,
    mut despawn: ResMut<DespawnResource>,
    mut resize: ResMut<ResizeResource>,
) {
    /*
     * Ground
     */
    let ground_size = 250.0;

    let entity = commands
        .spawn()
        .insert(Collider::cuboid(ground_size, 12.0))
        .id();
    despawn.entities.push(entity);

    commands
        .spawn_bundle(TransformBundle::from(Transform::from_xyz(
            ground_size,
            ground_size * 2.0,
            0.0,
        )))
        .insert(Collider::cuboid(12.0, ground_size * 2.0));

    commands
        .spawn_bundle(TransformBundle::from(Transform::from_xyz(
            -ground_size,
            ground_size * 2.0,
            0.0,
        )))
        .insert(Collider::cuboid(12.0, ground_size * 2.0));

    /*
     * Create the cubes
     */
    let num = 20;
    let rad = 5.0;

    let shift = rad * 2.0;
    let centerx = shift * (num as f32) / 2.0;
    let centery = shift / 2.0;

    for i in 0..num {
        for j in 0usize..num * 5 {
            let x = i as f32 * shift - centerx;
            let y = j as f32 * shift + centery + 2.0;

            let entity = commands
                .spawn_bundle(TransformBundle::from(Transform::from_xyz(x, y, 0.0))
                .insert(RigidBody::Dynamic)
                .insert(Collider::cuboid(rad, rad))
                .id();

            if (i + j * num) % 100 == 0 {
                resize.entities.push(entity);
            }
        }
    }
}

pub fn despawn(mut commands: Commands, time: Res<Time>, mut despawn: ResMut<DespawnResource>) {
    if time.seconds_since_startup() > 5.0 {
        for entity in &despawn.entities {
            println!("Despawning ground entity");
            commands.entity(*entity).despawn();
        }
        despawn.entities.clear();
    }
}

pub fn resize(mut commands: Commands, time: Res<Time>, mut resize: ResMut<ResizeResource>) {
    if time.seconds_since_startup() > 6.0 {
        for entity in &resize.entities {
            println!("Resizing a block");
            commands
                .entity(*entity)
                .insert(Collider::cuboid(20.0, 20.0));
        }
        resize.entities.clear();
    }
}
