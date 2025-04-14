use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

const START_POSITION: Vec3 = Vec3::new(0., 1.5, 0.);

fn main() {
    let mut app = App::new();
    app.add_plugins((
        DefaultPlugins,
        RapierPhysicsPlugin::<NoUserData>::default(),
        RapierDebugRenderPlugin::default(),
    ))
    .insert_resource(ClearColor(Color::srgb(0.2, 0.2, 0.2)))
    .add_systems(Startup, (spawn_basic_scene, spawn_shapes))
    .add_systems(FixedUpdate, update_cubes)
    .run();
}

#[derive(Component)]
struct Velocity(Vec3);
impl Velocity {
    pub fn random() -> Self {
        Velocity(
            // truly random velocity -> i found no matching vel that way
            // Vec3::new(
            //     fastrand::f32() * 2. - 1.,
            //     -fastrand::f32(),
            //     fastrand::f32() * 2. - 1.,
            // )
            // .normalize(),

            // this velocity always works
            //Vec3::new(1.0, -5.0, -1.0).normalize(),
            // integer velocities
            Vec3::new(
                fastrand::i32(-6..=6) as f32,
                fastrand::i32(-6..0) as f32,
                fastrand::i32(-6..=6) as f32,
            )
            .normalize(),
            // even a slight deviation from an integer velocity makes it very unlikely
            // (Vec3::new(
            //     fastrand::i32(-6..=6) as f32,
            //     fastrand::i32(-6..0) as f32,
            //     fastrand::i32(-6..=6) as f32,
            // )
            // .normalize()
            //     + Vec3::new(
            //         fastrand::f32() * 2. - 1.,
            //         -fastrand::f32(),
            //         fastrand::f32() * 2. - 1.,
            //     )
            //     .normalize()
            //         * 0.00001)
            //     .normalize(),
        )
    }
}

fn spawn_shapes(mut commands: Commands) {
    let group = CollisionGroups::new(Group::GROUP_1, Group::ALL ^ Group::GROUP_1);

    // every shape except cuboid works
    for shape in [
        Collider::cuboid(0.5, 0.5, 0.5),
        Collider::ball(0.5),
        Collider::capsule_y(0.5, 0.2),
        Collider::cylinder(0.5, 0.5),
    ] {
        // i spawned many shapes to get a better random sampling, but it also works with just one
        for _ in 0..100 {
            commands.spawn((
                group,
                Velocity::random(),
                RigidBody::KinematicPositionBased,
                shape.clone(),
                Transform::from_translation(START_POSITION),
                KinematicCharacterController {
                    filter_groups: Some(group),
                    slide: true, // sliding seems to make no difference
                    ..KinematicCharacterController::default()
                },
            ));
        }
    }
}

fn spawn_basic_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // spawn light
    commands.spawn((PointLight::default(), Transform::from_xyz(0., 5., 0.)));

    // spawn ground plane
    commands.spawn((
        // the size of the cuboid seems very important
        Collider::cuboid(10., 0.1, 10.),
        Mesh3d(meshes.add(Plane3d::new(Vec3::Y, Vec2::splat(10.)).mesh())),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.2, 0.9, 0.2),
            ..default()
        })),
    ));

    // add camera
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(10., 1.8, 0.).looking_at(Vec3::ZERO, Vec3::Y),
    ));
}

fn update_cubes(
    time: Res<Time>,
    mut cubes: Query<(
        &mut KinematicCharacterController,
        &mut Velocity,
        &mut Transform,
        &Collider,
    )>,
) {
    for (mut controller, mut velocity, mut transform, collider) in &mut cubes {
        let penetrated = transform.translation.y < -0.5;
        if penetrated || transform.translation.distance_squared(Vec3::ZERO) > 9. {
            if penetrated {
                println!(
                    " -> {:?}: {:?}, speed = {:?}",
                    collider,
                    velocity.0,
                    velocity.0.length()
                );
            }

            *transform = Transform::from_translation(START_POSITION);
            *velocity = Velocity::random();
            continue;
        }

        controller.translation = Some(velocity.0 * time.delta_secs());
    }
}
