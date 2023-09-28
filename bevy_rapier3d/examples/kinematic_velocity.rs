use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::default())
        .add_plugins(RapierDebugRenderPlugin::default())
        .insert_resource(RapierConfiguration::default())
        .add_systems(Startup, setup)
        .add_systems(Update, test)
        .run();
}

fn test(ctx: Res<RapierContext>, query: Query<(Entity, &Transform), With<RigidBody>>) {
    for (entity, transform) in &query {
        info!("transform: {:.1?}", transform.translation);
    }
}

fn setup(mut commands: Commands) {
    commands.spawn(Camera3dBundle {
        transform: Transform::from_xyz(-3.0, 3.0, 10.0).looking_at(Vec3::ZERO, Vec3::Y),
        ..Default::default()
    });

    commands
        .spawn(RigidBody::KinematicVelocityBased)
        .insert(Collider::ball(0.5))
        .insert(Velocity::linear(Vec3::Y * -1.81))
        .insert(TransformBundle::from(Transform::from_xyz(0.0, 1.0, 0.0)));
}
