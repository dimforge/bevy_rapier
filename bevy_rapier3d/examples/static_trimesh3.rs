use std::f32::consts::TAU;

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
        .add_plugin(RapierDebugRenderPlugin {
            enabled: true,
            ..Default::default()
        })
        .add_startup_system(setup_graphics)
        .add_startup_system(setup_physics)
        .insert_resource(BallState::default())
        .add_system(ball_spawner)
        .run();
}

fn setup_graphics(mut commands: Commands) {
    commands.spawn(Camera3dBundle {
        transform: Transform::from_xyz(-15.0, 8.0, 15.0)
            .looking_at(Vec3::new(-5.0, 0.0, 5.0), Vec3::Y),
        ..Default::default()
    });
}

fn ramp_size() -> Vec3 {
    Vec3::new(10.0, 1.0, 1.0)
}

pub fn setup_physics(mut commands: Commands) {
    // Create the ramp.
    let mut vertices: Vec<Vec3> = Vec::new();
    let mut indices: Vec<[u32; 3]> = Vec::new();
    let segments = 32;
    let ramp_size = ramp_size();

    for i in 0..=segments {
        // Half cosine wave vertically (with middle of low point at origin)
        let x = i as f32 / segments as f32 * ramp_size.x;
        let y = (-(i as f32 / segments as f32 * TAU / 2.0).cos() + 1.0) * ramp_size.y / 2.0;
        vertices.push(Vec3::new(x, y, -ramp_size.z / 2.0));
        vertices.push(Vec3::new(x, y, ramp_size.z / 2.0));
    }
    for i in 0..segments {
        // Two triangles making up a flat quad for each segment of the ramp.
        indices.push([2 * i, 2 * i + 1, 2 * i + 2]);
        indices.push([2 * i + 2, 2 * i + 1, 2 * i + 3]);
    }

    commands.spawn(Collider::trimesh(vertices, indices));

    // Create a bowl with a cosine cross-section,
    // so that we can join the end of the ramp smoothly
    // to the lip of the bowl.
    let mut vertices: Vec<Vec3> = Vec::new();
    let mut indices: Vec<[u32; 3]> = Vec::new();

    let segments = 32;
    let bowl_size = Vec3::new(10.0, 3.0, 10.0);

    for ix in 0..=segments {
        for iz in 0..=segments {
            // Map x and y into range [-1.0, 1.0];
            let shifted_z = (iz as f32 / segments as f32 - 0.5) * 2.0;
            let shifted_x = (ix as f32 / segments as f32 - 0.5) * 2.0;
            // Clamp radius at 1.0 or lower so the bowl has a flat lip near the corners.
            let clamped_radius = (shifted_z.powi(2) + shifted_x.powi(2)).sqrt().min(1.0);
            let x = shifted_x * bowl_size.x / 2.0;
            let z = shifted_z * bowl_size.z / 2.0;
            let y = ((clamped_radius - 0.5) * TAU / 2.0).sin() * bowl_size.y / 2.0;
            vertices.push(Vec3::new(x, y, z));
        }
    }
    for ix in 0..segments {
        for iz in 0..segments {
            // Start of the two relevant rows of vertices.
            let row0 = ix * (segments + 1);
            let row1 = (ix + 1) * (segments + 1);
            // Two triangles making up a not-very-flat quad for each segment of the bowl.
            indices.push([row0 + iz, row0 + iz + 1, row1 + iz]);
            indices.push([row1 + iz, row0 + iz + 1, row1 + iz + 1]);
        }
    }
    // Position so ramp connects smoothly
    // to one edge of the lip of the bowl.
    commands.spawn((
        TransformBundle::from(Transform::from_xyz(
            -bowl_size.x / 2.0,
            -bowl_size.y / 2.0,
            bowl_size.z / 2.0 - ramp_size.z / 2.0,
        )),
        Collider::trimesh(vertices, indices),
    ));
}

#[derive(Resource)]
struct BallState {
    seconds_until_next_spawn: f32,
    seconds_between_spawns: f32,
    balls_spawned: usize,
    max_balls: usize,
}

impl Default for BallState {
    fn default() -> Self {
        Self {
            seconds_until_next_spawn: 0.5,
            seconds_between_spawns: 2.0,
            balls_spawned: 0,
            max_balls: 10,
        }
    }
}

fn ball_spawner(
    mut commands: Commands,
    rapier_context: Res<RapierContext>,
    mut ball_state: ResMut<BallState>,
) {
    if ball_state.balls_spawned >= ball_state.max_balls {
        return;
    }

    // NOTE: The timing here only works properly with `time_dependent_number_of_timesteps`
    // disabled, as it is for examples.
    ball_state.seconds_until_next_spawn -= rapier_context.integration_parameters.dt;
    if ball_state.seconds_until_next_spawn > 0.0 {
        return;
    }
    ball_state.seconds_until_next_spawn = ball_state.seconds_between_spawns;

    // Spawn a ball near the top of the ramp.
    let ramp_size = ramp_size();
    let rad = 0.3;

    commands.spawn((
        TransformBundle::from(Transform::from_xyz(
            ramp_size.x * 0.9,
            ramp_size.y / 2.0 + rad * 3.0,
            0.0,
        )),
        RigidBody::Dynamic,
        Collider::ball(rad),
        Restitution::new(0.5),
    ));

    ball_state.balls_spawned += 1;
}
