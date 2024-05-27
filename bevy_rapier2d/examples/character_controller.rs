use bevy::prelude::*;
use bevy_inspector_egui::quick::WorldInspectorPlugin;
use bevy_rapier2d::prelude::*;

#[derive(Component)]
struct Player;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::pixels_per_meter(100.0))
        .add_plugins(RapierDebugRenderPlugin::default())
        .add_plugins(WorldInspectorPlugin::new())
        .add_systems(Startup, setup)
        .add_systems(Update, update)
        .run();
}

fn setup(mut commands: Commands) {
    commands.spawn(Camera2dBundle::default());

    // player
    commands
        .spawn(RigidBody::KinematicPositionBased)
        .insert(Collider::cuboid(24.0, 18.0))
        .insert(KinematicCharacterController::default())
        .insert(SpatialBundle::default())
        .insert(TransformBundle::from(Transform::from_xyz(0.0, 0.0, 0.0)))
        .insert(Name::new("Player"))
        .insert(Sleeping::disabled())
        .insert(GravityScale(1f32))
        .insert(KinematicCharacterControllerOutput::default())
        .insert(Player);

    // ground
    commands
        .spawn(Collider::cuboid(500.0, 50.0))
        .insert(Name::new("Ground"))
        .insert(TransformBundle::from(Transform::from_xyz(0.0, -100.0, 0.0)));
}

fn update(query: Query<(&Player, &KinematicCharacterControllerOutput)>) {
    if query.is_empty() {
        return;
    }

    let (_, output) = query.single();

    if output.grounded {
        println!("player grounded");
    }
}
