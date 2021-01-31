use bevy::prelude::*;
use bevy_rapier2d::physics::{RapierConfiguration, RapierPhysicsPlugin, RigidBodyHandleComponent};
use bevy_rapier2d::rapier::dynamics::{RigidBodyBuilder, RigidBodySet};
use bevy_rapier2d::rapier::geometry::ColliderBuilder;
use bevy_rapier2d::rapier::na::Vector2;

fn main() {
    App::build()
        .add_resource(WindowDescriptor {
            title: "Player Movement Example".to_string(),
            width: 1000.0,
            height: 1000.0,
            ..Default::default()
        })
        .add_plugins(DefaultPlugins)
        .add_plugin(bevy_winit::WinitPlugin::default())
        .add_plugin(bevy_wgpu::WgpuPlugin::default())
        .add_startup_system(spawn_player.system())
        .add_system(player_movement.system())
        .add_plugin(RapierPhysicsPlugin)
        .run();
}

// The float value is the player movemnt speed in 'pixels/second'.
struct Player(f32);

fn spawn_player(
    commands: &mut Commands,
    mut materials: ResMut<Assets<ColorMaterial>>,
    mut rapier_config: ResMut<RapierConfiguration>,
) {
    // Set gravity to 0.0 and spawn camera.
    rapier_config.gravity = Vector2::zeros();
    commands.spawn(Camera2dBundle::default());

    let sprite_size_x = 40.0;
    let sprite_size_y = 40.0;

    // While we want our sprite to look ~40 px square, we want to keep the physics units smaller
    // to prevent float rounding problems. To do this, we set the scale factor in RapierConfiguration
    // and divide our sprite_size by the scale.
    rapier_config.scale = 20.0;
    let collider_size_x = sprite_size_x / rapier_config.scale;
    let collider_size_y = sprite_size_y / rapier_config.scale;

    // Spawn entity with `Player` struct as a component for access in movement query.
    commands
        .spawn(SpriteBundle {
            material: materials.add(Color::rgb(0.0, 0.0, 0.0).into()),
            sprite: Sprite::new(Vec2::new(sprite_size_x, sprite_size_y)),
            ..Default::default()
        })
        .with(RigidBodyBuilder::new_dynamic())
        .with(ColliderBuilder::cuboid(
            collider_size_x / 2.0,
            collider_size_y / 2.0,
        ))
        .with(Player(300.0));
}

fn player_movement(
    keyboard_input: Res<Input<KeyCode>>,
    rapier_parameters: Res<RapierConfiguration>,
    mut rigid_bodies: ResMut<RigidBodySet>,
    player_info: Query<(&Player, &RigidBodyHandleComponent)>,
) {
    for (player, rigid_body_component) in player_info.iter() {
        let x_axis = -(keyboard_input.pressed(KeyCode::A) as i8)
            + (keyboard_input.pressed(KeyCode::D) as i8);
        let y_axis = -(keyboard_input.pressed(KeyCode::S) as i8)
            + (keyboard_input.pressed(KeyCode::W) as i8);

        let mut move_delta = Vector2::new(x_axis as f32, y_axis as f32);
        if move_delta != Vector2::zeros() {
            // Note that the RapierConfiguration::Scale factor is also used here to transform
            // the move_delta from: 'pixels/second' to 'physics_units/second'
            move_delta /= move_delta.magnitude() * rapier_parameters.scale;
        }

        // Update the velocity on the rigid_body_component,
        // the bevy_rapier plugin will update the Sprite transform.
        if let Some(rb) = rigid_bodies.get_mut(rigid_body_component.handle()) {
            rb.set_linvel(move_delta * player.0, true);
        }
    }
}
