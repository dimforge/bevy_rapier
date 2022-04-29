use bevy::prelude::*;
use bevy_rapier2d::prelude::*;

fn main() {
    App::new()
        .insert_resource(ClearColor(Color::rgb(
            0xF9 as f32 / 255.0,
            0xF9 as f32 / 255.0,
            0xFF as f32 / 255.0,
        )))
        .insert_resource(Msaa::default())
        .add_plugins(DefaultPlugins)
        .add_plugin(RapierPhysicsPlugin::<NoUserData>::pixels_per_meter(100.0))
        .add_plugin(RapierDebugRenderPlugin::default())
        .add_startup_system(setup_graphics)
        .add_startup_system(setup_physics)
        .add_system_to_stage(CoreStage::PostUpdate, display_events)
        .run();
}

fn setup_graphics(mut commands: Commands) {
    commands.spawn_bundle(OrthographicCameraBundle::new_2d());
}

fn display_events(mut collision_events: EventReader<CollisionEvent>) {
    for collision_event in collision_events.iter() {
        println!("Received collision event: {:?}", collision_event);
    }
}

pub fn setup_physics(mut commands: Commands) {
    /*
     * Ground
     */
    commands
        .spawn()
        .insert(Collider::cuboid(80.0, 20.0))
        .insert(Transform::from_xyz(0.0, -24.0, 0.0));

    commands
        .spawn()
        .insert(Collider::cuboid(80.0, 30.0))
        .insert(Sensor)
        .insert(Transform::from_xyz(0.0, 100.0, 0.0));

    commands
        .spawn()
        .insert(RigidBody::Dynamic)
        .insert(Collider::cuboid(10.0, 10.0))
        .insert(Transform::from_xyz(0.0, 260.0, 0.0))
        .insert(ActiveEvents::COLLISION_EVENTS);
}
