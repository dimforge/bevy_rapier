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
        .add_startup_system(setup_simulation)
        // .add_system(update_cam_pos)
        .run();
}

const VEL_Y: f32 = 1000.0;
const TOP_CUBE_DIFF_VEL: f32 = -1.0;

#[derive(Component)]
struct FollowMe;

fn setup_simulation(mut commands: Commands) {
    // Right side - independent entities

    commands
        .spawn((
            TransformBundle::from(Transform::from_xyz(1.0, 0.0, 0.0)),
            RigidBody::Dynamic,
            Collider::cuboid(0.5, 0.5, 0.5),
            ColliderDebugColor(Color::hsl(180.0, 1.0, 0.3)),
            Velocity {
                linvel: Vec3::new(0.0, VEL_Y, 0.0),
                angvel: Vec3::new(0.0, 0.0, 0.0),
            },
            GravityScale(0.0),
            Ccd::enabled(),
        ))
        .with_children(|child| {
            child.spawn((
                TransformBundle::from(Transform::from_xyz(0.0, 5.0, 0.0)),
                RigidBody::Dynamic,
                Collider::cuboid(0.5, 0.5, 0.5),
                ColliderDebugColor(Color::hsl(220.0, 1.0, 0.3)),
                Ccd::enabled(),
                FollowMe,
                Velocity::linear(Vec3::new(0.0, TOP_CUBE_DIFF_VEL, 0.0)),
                GravityScale(0.0),
            ));

            child.spawn(Camera3dBundle {
                transform: Transform::from_xyz(0.0, 10.0, 10.0)
                    .looking_at(Vec3::new(0.0, 0.0, 0.0), Vec3::Y),
                ..Default::default()
            });
        });

    // Left side - independent entities are fine

    commands.spawn((
        TransformBundle::from(Transform::from_xyz(-1.0, 0.0, 0.0)),
        RigidBody::Dynamic,
        Collider::cuboid(0.5, 0.5, 0.5),
        ColliderDebugColor(Color::hsl(180.0, 1.0, 0.3)),
        Velocity {
            linvel: Vec3::new(0.0, VEL_Y, 0.0),
            angvel: Vec3::new(0.0, 0.0, 0.0),
        },
        GravityScale(0.0),
        Ccd::enabled(),
    ));

    commands.spawn((
        TransformBundle::from(Transform::from_xyz(-1.0, 5.0, 0.0)),
        RigidBody::Dynamic,
        Collider::cuboid(0.5, 0.5, 0.5),
        ColliderDebugColor(Color::hsl(220.0, 1.0, 0.3)),
        Ccd::enabled(),
        Velocity {
            linvel: Vec3::new(0.0, VEL_Y + TOP_CUBE_DIFF_VEL, 0.0),
            angvel: Vec3::new(0.0, 0.0, 0.0),
        },
        GravityScale(0.0),
    ));
}

fn update_cam_pos(
    follow_me: Query<&GlobalTransform, (Without<Camera>, With<FollowMe>)>,
    mut cam_trans: Query<&mut Transform, With<Camera>>,
) {
    let trans = follow_me.get_single().unwrap().translation();
    cam_trans.get_single_mut().unwrap().translation = Vec3::new(0.0, trans.y + 10.0, 10.0);
}
