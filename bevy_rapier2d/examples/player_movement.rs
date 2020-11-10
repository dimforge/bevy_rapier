use bevy::prelude::*;
use bevy_rapier2d::physics::{RapierConfiguration, RapierPhysicsPlugin, RigidBodyHandleComponent};
use bevy_rapier2d::rapier::dynamics::{RigidBodyBuilder, RigidBodySet};
use bevy_rapier2d::rapier::geometry::ColliderBuilder;
use bevy_rapier2d::rapier::na::Vector2;

fn main() {
    App::build()
        .add_resource(WindowDescriptor {
            title: "Player Movement Example".to_string(),
            width: 1000,
            height: 1000,
            ..Default::default()
        })
        .add_startup_system(spawn_player.system())
        .add_system(player_movement.system())
        .add_plugin(RapierPhysicsPlugin)
        .add_plugins(DefaultPlugins)
        .run();
}

// The float value is essentially a player movement speed factor.
struct Player(f32);

fn spawn_player(
    mut commands: Commands,
    mut materials: ResMut<Assets<ColorMaterial>>,
    mut rapier_config: ResMut<RapierConfiguration>,
) {
    // Set gravity to 0.0 and spawn camera.
    rapier_config.gravity = Vector2::zeros();
    commands.spawn(Camera2dComponents::default());

    let sprite_size_x = 40.0;
    let sprite_size_y = 40.0;

    // Spawn entity with `Player` struct as a component for access in movement query.
    // If `can_sleep` is not set to false, entity will not respond to input after idle time.
    commands
        .spawn(SpriteComponents {
            material: materials.add(Color::rgb(0.0, 0.0, 0.0).into()),
            sprite: Sprite::new(Vec2::new(sprite_size_x, sprite_size_y)),
            ..Default::default()
        })
        .with(RigidBodyBuilder::new_dynamic().can_sleep(false))
        .with(ColliderBuilder::cuboid(
            sprite_size_x / 2.0,
            sprite_size_y / 2.0,
        ))
        .with(Player(300.0));
}

fn player_movement(
    keyboard_input: Res<Input<KeyCode>>,
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
            move_delta /= move_delta.magnitude();
        }

        // Write player rigid_body velocity directly to component,
        // the bevy_rapier plugin will update the Sprite transform.
        if let Some(mut rb) = rigid_bodies.get_mut(rigid_body_component.handle()) {
            rb.linvel = move_delta * player.0;
        }
    }
}
