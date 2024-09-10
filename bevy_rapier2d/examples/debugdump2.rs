use bevy::prelude::*;
use bevy_mod_debugdump::{schedule_graph, schedule_graph_dot};
use bevy_rapier2d::prelude::*;

fn main() {
    let mut app = App::new();
    app.insert_resource(ClearColor(Color::srgb(
        0xF9 as f32 / 255.0,
        0xF9 as f32 / 255.0,
        0xFF as f32 / 255.0,
    )))
    .add_plugins((
        DefaultPlugins,
        RapierPhysicsPlugin::<NoUserData>::pixels_per_meter(100.0),
        RapierDebugRenderPlugin::default(),
    ))
    .add_systems(Startup, (setup_graphics, setup_physics))
    .add_systems(PostUpdate, display_events.after(PhysicsSet::StepSimulation));

    let mut debugdump_settings = schedule_graph::Settings::default();
    // Filter out some less relevant systems.
    debugdump_settings.include_system =
        Some(Box::new(|system: &(dyn System<In = (), Out = ()>)| {
            if system.name().starts_with("bevy_pbr")
                || system.name().starts_with("bevy_render")
                || system.name().starts_with("bevy_gizmos")
                || system.name().starts_with("bevy_winit")
                || system.name().starts_with("bevy_sprite")
            {
                return dbg!(false);
            }
            true
        }));
    let dot = schedule_graph_dot(&mut app, PostUpdate, &debugdump_settings);
    println!("{dot}");
}

pub fn setup_graphics(mut commands: Commands) {
    commands.spawn(Camera2dBundle::default());
}

pub fn display_events(
    mut collision_events: EventReader<CollisionEvent>,
    mut contact_force_events: EventReader<ContactForceEvent>,
) {
    for collision_event in collision_events.read() {
        println!("Received collision event: {collision_event:?}");
    }

    for contact_force_event in contact_force_events.read() {
        println!("Received contact force event: {contact_force_event:?}");
    }
}

pub fn setup_physics(mut commands: Commands) {
    /*
     * Ground
     */
    commands.spawn((
        TransformBundle::from(Transform::from_xyz(0.0, -24.0, 0.0)),
        Collider::cuboid(80.0, 20.0),
    ));

    commands.spawn((
        TransformBundle::from(Transform::from_xyz(0.0, 100.0, 0.0)),
        Collider::cuboid(80.0, 30.0),
        Sensor,
    ));

    commands.spawn((
        TransformBundle::from(Transform::from_xyz(0.0, 260.0, 0.0)),
        RigidBody::Dynamic,
        Collider::cuboid(10.0, 10.0),
        ActiveEvents::COLLISION_EVENTS,
        ContactForceEventThreshold(10.0),
    ));
}
