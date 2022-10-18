use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugin(RapierPhysicsPlugin::<NoUserData>::default())
        .add_plugin(RapierDebugRenderPlugin::default())
        .add_startup_system(setup_level)
        .add_startup_system(spawn_player)
        .add_system(move_player)
        .run();
}

fn spawn_player(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let defaults = bevy_rapier3d::control::KinematicCharacterController::default();
    commands
        .spawn()
        .insert_bundle(PbrBundle {
            mesh: meshes.add(Mesh::from(shape::Box::new(0.5, 1.8, 0.5))),
            material: materials.add(Color::BLUE.into()),
            ..default()
        })
        .insert(Collider::cylinder(0.9, 0.5))
        //.insert(Collider::capsule_y(0.5, 0.5))
        .insert(RigidBody::KinematicVelocityBased)
        .insert(KinematicCharacterController {
            translation: Some(Vec3::ZERO),
            custom_shape: None,
            custom_mass: None,
            up: Vec3::Y,
            offset: CharacterLength::Absolute(0.12),
            slide: false,
            autostep: Some(CharacterAutostep {
                max_height: CharacterLength::Absolute(0.2),
                min_width: CharacterLength::Absolute(0.1),
                include_dynamic_bodies: false,
            }),
            max_slope_climb_angle: 45.0_f32.to_radians(),
            min_slope_slide_angle: defaults.min_slope_slide_angle,
            apply_impulse_to_dynamic_bodies: false,
            snap_to_ground: Some(CharacterLength::Absolute(0.12)),
            filter_flags: QueryFilterFlags::empty(),
            filter_groups: None,
        })
        .insert(Transform::from_xyz(0.0, 1.0, 0.0));
}

fn setup_level(
    mut commands: Commands,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    commands.spawn_bundle(Camera3dBundle {
        transform: Transform::from_xyz(0.0, 6., 12.0).looking_at(Vec3::new(0., 1., 0.), Vec3::Y),
        ..default()
    });

    commands.spawn_bundle(PointLightBundle {
        point_light: PointLight {
            intensity: 9000.0,
            range: 100.,
            shadows_enabled: true,
            ..default()
        },
        transform: Transform::from_xyz(8.0, 16.0, 8.0),
        ..default()
    });

    // ground
    commands
        .spawn()
        .insert_bundle(PbrBundle {
            mesh: meshes.add(Mesh::from(shape::Box::new(20., 0.4, 20.0))),
            material: materials.add(Color::RED.into()),
            ..default()
        })
        .insert(Collider::cuboid(10., 0.2, 10.0))
        .insert(RigidBody::Fixed)
        .insert_bundle(SpatialBundle::from_transform(Transform::from_xyz(
            0.0, -0.4, 0.0,
        )));

    commands
        .spawn()
        .insert_bundle(PbrBundle {
            mesh: meshes.add(Mesh::from(shape::Box::new(1.0, 1.0, 1.0))),
            material: materials.add(Color::YELLOW.into()),
            ..default()
        })
        .insert(Collider::cuboid(0.5, 0.5, 0.5))
        .insert(RigidBody::Fixed)
        .insert_bundle(SpatialBundle::from_transform(Transform::from_xyz(
            5.0, 0.0, 3.0,
        )));

    commands
        .spawn()
        .insert_bundle(PbrBundle {
            mesh: meshes.add(Mesh::from(shape::Box::new(1.0, 1.0, 1.0))),
            material: materials.add(Color::YELLOW.into()),
            ..default()
        })
        .insert(Collider::cuboid(0.5, 0.5, 0.5))
        .insert(RigidBody::Fixed)
        .insert_bundle(SpatialBundle::from_transform(Transform::from_xyz(
            -5.0, 0.5, 3.0,
        )));

    let mut transform = Transform {
        translation: Vec3::new(-8.0, -0.5, -2.0),
        rotation: Quat::from_rotation_z(45.0_f32.to_radians()),
        ..default()
    };
    for _ in 0..=6 {
        transform.translation += Vec3::new(2.0, 0.0, 0.0);
        commands
            .spawn()
            .insert_bundle(PbrBundle {
                mesh: meshes.add(Mesh::from(shape::Box::new(1.0, 1.0, 3.0))),
                material: materials.add(Color::GREEN.into()),
                ..default()
            })
            .insert(Collider::cuboid(0.5, 0.5, 1.5))
            .insert(RigidBody::Fixed)
            .insert_bundle(SpatialBundle::from_transform(transform));
    }

    let pos = Vec3::new(-5.0, -0.1, -8.0);
    let mut transform = Transform::from_xyz(0.0, 0.0, 0.0);
    for i in 0..=10 {
        transform.translation = pos + (Vec3::new(1.0, 0.15, 0.0) * i as f32);
        commands
            .spawn()
            .insert_bundle(PbrBundle {
                mesh: meshes.add(Mesh::from(shape::Box::new(1.0, 0.2, 3.0))),
                material: materials.add(Color::GREEN.into()),
                ..default()
            })
            .insert(Collider::cuboid(0.5, 0.1, 1.5))
            .insert(RigidBody::Fixed)
            .insert_bundle(SpatialBundle::from_transform(transform));
    }

    commands
        .spawn()
        .insert_bundle(PbrBundle {
            mesh: meshes.add(Mesh::from(shape::UVSphere {
                radius: 1.0,
                sectors: 32,
                stacks: 4,
            })),
            material: materials.add(Color::GREEN.into()),
            ..default()
        })
        .insert(Collider::ball(1.0))
        .insert(RigidBody::Fixed)
        .insert(Transform::from_xyz(0.0, -0.7, -6.0));
}

fn move_player(
    time: Res<Time>,
    input: Res<Input<KeyCode>>,
    mut controller_query: Query<(
        &mut KinematicCharacterController,
        Option<&KinematicCharacterControllerOutput>,
    )>,
) {
    let mut gravity = Vec3::Y * -9.82;
    let acceleration_direction = Vec3::new(
        (input.pressed(KeyCode::D) as i8 - input.pressed(KeyCode::A) as i8) as f32,
        0.0,
        (input.pressed(KeyCode::S) as i8 - input.pressed(KeyCode::W) as i8) as f32,
    )
    .normalize_or_zero()
        * 9.0;
    for (mut controller, optional_output) in controller_query.iter_mut() {
        if let Some(output) = optional_output {
            if output.grounded {
                gravity = Vec3::ZERO;
            } else {
                gravity += output.effective_translation.y;
            }
        }
        controller.translation = Some((acceleration_direction + gravity) * time.delta_seconds());
    }
}
