use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

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
        .run();
}

fn setup_compound(mut commands: Commands, mut meshes: ResMut<Assets<Mesh>>) {
    // Normal compound
    // - Scaling works as expected
    commands
        .spawn(TransformBundle::default())
        .insert(Name::new("Compound"))
        .insert(RigidBody::Dynamic)
        .insert(ToggleCollider)
        .insert(ToggleScale)
        .insert(Collider::compound(vec![
            (Vec3::ZERO, Quat::IDENTITY, Collider::cuboid(0.5, 0.5, 0.5)),
            (
                Vec3::new(0.0, 1.0, 0.0),
                Quat::IDENTITY,
                Collider::cuboid(0.5, 0.5, 0.5),
            ),
            (
                Vec3::new(-1.0, 1.0, 0.0),
                Quat::IDENTITY,
                Collider::cuboid(0.5, 0.5, 0.5),
            ),
        ]));

    let mesh = meshes.add(Mesh::from(shape::UVSphere {
        radius: 0.2,
        ..default()
    }));

    commands
        .spawn(PbrBundle {
            mesh: mesh.clone(),
            ..default()
        })
        .insert(TransformBundle::from_transform(Transform {
            translation: Vec3::new(-4.0, 0.0, 0.0),
            ..default()
        }))
        .insert(Name::new("Standalone collider"))
        .insert(ToggleScale)
        .insert(ToggleCollider)
        .insert(Collider::cuboid(0.5, 0.5, 0.5));

    commands
        .spawn(PbrBundle {
            mesh: mesh.clone(),
            ..default()
        })
        .insert(TransformBundle::from_transform(Transform {
            translation: Vec3::new(4.0, 0.0, 0.0),
            ..default()
        }))
        .insert(Name::new("Compound via children"))
        .insert(RigidBody::Dynamic)
        .insert(ToggleScale)
        //.insert(ToggleCollider)
        .insert(Collider::cuboid(0.5, 0.5, 0.5))
        .with_children(|children| {
            children
                .spawn(PbrBundle {
                    mesh: mesh.clone(),
                    ..default()
                })
                .insert(TransformBundle::from_transform(Transform {
                    translation: Vec3::new(0.0, 1.0, 0.0),
                    ..default()
                }))
                .insert(Name::new("Child collider"))
                .insert(Collider::cuboid(0.5, 0.5, 0.5));

            children
                .spawn(PbrBundle {
                    mesh: mesh.clone(),
                    ..default()
                })
                .insert(TransformBundle::from_transform(Transform {
                    translation: Vec3::new(-1.0, 1.0, 0.0),
                    ..default()
                }))
                .insert(Name::new("Child collider"))
                .insert(Collider::cuboid(0.5, 0.5, 0.5));
        });
}

#[derive(Component)]
pub struct ToggleCollider;
#[derive(Component)]
pub struct ToggleScale;

fn toggle_compound(
    keycode: Res<Input<KeyCode>>,
    mut commands: Commands,
    mut collider: Query<(Entity, &mut Collider), With<ToggleCollider>>,
    mut scale: Query<(Entity, &mut Transform), With<ToggleScale>>,
) {
    let new_collider = if keycode.just_pressed(KeyCode::Key1) {
        Some(Collider::compound(vec![
            (Vec3::ZERO, Quat::IDENTITY, Collider::cuboid(0.5, 0.5, 0.5)),
            (
                Vec3::new(0.0, 1.0, 0.0),
                Quat::IDENTITY,
                Collider::cuboid(0.5, 0.5, 0.5),
            ),
            (
                Vec3::new(-1.0, 1.0, 0.0),
                Quat::IDENTITY,
                Collider::cuboid(0.5, 0.5, 0.5),
            ),
        ]))
    } else if keycode.just_pressed(KeyCode::Key2) {
        Some(Collider::compound(vec![
            (Vec3::ZERO, Quat::IDENTITY, Collider::ball(0.5)),
            (
                Vec3::new(0.0, 1.0, 0.0),
                Quat::IDENTITY,
                Collider::ball(0.5),
            ),
            (
                Vec3::new(-1.0, 1.0, 0.0),
                Quat::IDENTITY,
                Collider::ball(0.5),
            ),
        ]))
    } else {
        None
    };

    let new_scale_ratio = if keycode.just_pressed(KeyCode::T) {
        Some(1.1)
    } else if keycode.just_pressed(KeyCode::R) {
        Some(0.9)
    } else {
        None
    };

    if let Some(new_scale_ratio) = new_scale_ratio {
        for (entity, mut transform) in &mut scale {
            transform.scale *= new_scale_ratio;
            transform.scale = transform.scale.max(Vec3::new(0.1, 0.1, 0.1));
            transform.scale = transform.scale.min(Vec3::new(5.0, 5.0, 5.0));
        }
    }

    if let Some(new_collider) = new_collider {
        for (entity, mut collider) in &mut collider {
            *collider = new_collider.clone();
        }
    }
}

fn setup_graphics(mut commands: Commands) {
    commands.spawn(Camera3dBundle {
        transform: Transform::from_xyz(-30.0, 30.0, 100.0)
            .looking_at(Vec3::new(0.0, 10.0, 0.0), Vec3::Y),
        ..Default::default()
    });
}

pub fn setup_physics(mut commands: Commands) {
    /*
     * Ground
     */
    let ground_size = 200.1;
    let ground_height = 0.1;

    commands.spawn((
        TransformBundle::from(Transform::from_xyz(0.0, -ground_height, 0.0)),
        Collider::cuboid(ground_size, ground_height, ground_size),
    ));

    /*
     * Create the cubes
     */
    let num = 0;
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

    for j in 0usize..num {
        for i in 0..num {
            for k in 0usize..num {
                let x = i as f32 * shift - centerx + offset;
                let y = j as f32 * shift + centery + 3.0;
                let z = k as f32 * shift - centerz + offset;
                color += 1;

                commands
                    .spawn(TransformBundle::from(Transform::from_rotation(
                        Quat::from_rotation_x(0.2),
                    )))
                    .with_children(|child| {
                        child.spawn((
                            TransformBundle::from(Transform::from_xyz(x, y, z)),
                            RigidBody::Dynamic,
                            Collider::cuboid(rad, rad, rad),
                            ColliderDebugColor(colors[color % 3]),
                        ));
                    });
            }
        }

        offset -= 0.05 * rad * (num as f32 - 1.0);
    }
}
