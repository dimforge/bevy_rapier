use bevy::prelude::*;
use bevy_rapier2d::prelude::*;

use crate::{cleanup_resource, ExampleResource, Examples};

#[derive(Default)]
pub struct ExamplePluginDespawn2;

impl Plugin for ExamplePluginDespawn2 {
    fn build(&self, app: &mut App) {
        app.init_resource::<DespawnResource>()
            .init_resource::<ResizeResource>()
            .add_systems(OnEnter(Examples::Despawn2), (setup_graphics, setup_physics))
            .add_systems(
                Update,
                (despawn, resize).run_if(in_state(Examples::Despawn2)),
            )
            .add_systems(OnExit(Examples::Despawn2), (cleanup, cleanup_resource));
    }
}

#[derive(Resource, Default)]
struct DespawnResource {
    entities: Vec<Entity>,
    timer: Timer,
}

#[derive(Resource, Default)]
struct ResizeResource {
    entities: Vec<Entity>,
    timer: Timer,
}

fn cleanup(mut despawn: ResMut<DespawnResource>, mut resize: ResMut<ResizeResource>) {
    despawn.entities.clear();
    resize.entities.clear();
}

fn setup_graphics(
    mut commands: Commands,
    mut res: ResMut<ExampleResource>,
    mut despawn: ResMut<DespawnResource>,
    mut resize: ResMut<ResizeResource>,
) {
    //reset timers
    resize.timer = Timer::from_seconds(6.0, TimerMode::Once);
    despawn.timer = Timer::from_seconds(5.0, TimerMode::Once);

    commands.insert_resource(ClearColor(Color::rgb(
        0xF9 as f32 / 255.0,
        0xF9 as f32 / 255.0,
        0xFF as f32 / 255.0,
    )));

    let camera = commands
        .spawn(Camera2dBundle {
            transform: Transform::from_xyz(0.0, 20.0, 0.0),
            ..default()
        })
        .id();
    res.camera = Some(camera);
}

fn setup_physics(
    mut commands: Commands,
    mut despawn: ResMut<DespawnResource>,
    mut resize: ResMut<ResizeResource>,
    mut res: ResMut<ExampleResource>,
) {
    let root = commands
        .spawn(TransformBundle::default())
        .with_children(|parent| {
            /*
             * Ground
             */
            let ground_size = 250.0;

            let entity = parent.spawn(Collider::cuboid(ground_size, 12.0)).id();
            despawn.entities.push(entity);

            parent.spawn((
                TransformBundle::from(Transform::from_xyz(ground_size, ground_size * 2.0, 0.0)),
                Collider::cuboid(12.0, ground_size * 2.0),
            ));

            parent.spawn((
                TransformBundle::from(Transform::from_xyz(-ground_size, ground_size * 2.0, 0.0)),
                Collider::cuboid(12.0, ground_size * 2.0),
            ));

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

                    let entity = parent
                        .spawn((
                            TransformBundle::from(Transform::from_xyz(x, y, 0.0)),
                            RigidBody::Dynamic,
                            Collider::cuboid(rad, rad),
                        ))
                        .id();

                    if (i + j * num) % 100 == 0 {
                        resize.entities.push(entity);
                    }
                }
            }
        })
        .id();

    res.root = Some(root);
}

fn despawn(mut commands: Commands, time: Res<Time>, mut despawn: ResMut<DespawnResource>) {
    if despawn.timer.tick(time.delta()).finished() {
        for entity in &despawn.entities {
            println!("Despawning ground entity");
            commands.entity(*entity).remove_parent();
            commands.entity(*entity).despawn_recursive();
        }
        despawn.entities.clear();
    }
}

fn resize(mut commands: Commands, time: Res<Time>, mut resize: ResMut<ResizeResource>) {
    if resize.timer.tick(time.delta()).finished() {
        for entity in &resize.entities {
            println!("Resizing a block");
            commands
                .entity(*entity)
                .insert(Collider::cuboid(20.0, 20.0));
        }
        resize.entities.clear();
    }
}
