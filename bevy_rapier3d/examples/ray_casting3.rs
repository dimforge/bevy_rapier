extern crate rapier3d as rapier; // For the debug UI.

use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

use bevy::render::camera::Camera;
use rapier::geometry::{ColliderShape, InteractionGroups, Ray};
use rapier::pipeline::{PhysicsPipeline, QueryPipeline};
use ui::DebugUiPlugin;

#[path = "../../src_debug_ui/mod.rs"]
mod ui;

fn main() {
    App::new()
        .insert_resource(ClearColor(Color::rgb(
            0xF9 as f32 / 255.0,
            0xF9 as f32 / 255.0,
            0xFF as f32 / 255.0,
        )))
        .insert_resource(Msaa::default())
        .add_plugins(DefaultPlugins)
        .add_plugin(RapierPhysicsPlugin::<NoUserData>::default())
        .add_plugin(RapierRenderPlugin)
        .add_plugin(DebugUiPlugin)
        .add_startup_system(setup_graphics.system())
        .add_startup_system(setup_physics.system())
        .add_startup_system(enable_physics_profiling.system())
        .add_system(cast_ray.system())
        .run();
}

fn enable_physics_profiling(mut pipeline: ResMut<PhysicsPipeline>) {
    pipeline.counters.enable()
}

fn setup_graphics(mut commands: Commands) {
    const HALF_SIZE: f32 = 100.0;

    commands.spawn_bundle(DirectionalLightBundle {
        directional_light: DirectionalLight {
            illuminance: 10000.0,
            // Configure the projection to better fit the scene
            shadow_projection: OrthographicProjection {
                left: -HALF_SIZE,
                right: HALF_SIZE,
                bottom: -HALF_SIZE,
                top: HALF_SIZE,
                near: -10.0 * HALF_SIZE,
                far: 100.0 * HALF_SIZE,
                ..Default::default()
            },
            shadows_enabled: true,
            ..Default::default()
        },
        transform: Transform {
            translation: Vec3::new(10.0, 2.0, 10.0),
            rotation: Quat::from_rotation_x(-std::f32::consts::FRAC_PI_4),
            ..Default::default()
        },
        ..Default::default()
    });
    commands.spawn_bundle(PerspectiveCameraBundle {
        transform: Transform::from_matrix(Mat4::face_toward(
            Vec3::new(-30.0, 30.0, 100.0),
            Vec3::new(0.0, 10.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        )),
        ..Default::default()
    });
}

pub fn setup_physics(mut commands: Commands) {
    /*
     * Ground
     */
    let ground_size = 200.1;
    let ground_height = 0.1;

    let collider = ColliderBundle {
        shape: ColliderShape::cuboid(ground_size, ground_height, ground_size).into(),
        position: [0.0, -ground_height, 0.0].into(),
        ..ColliderBundle::default()
    };

    commands
        .spawn_bundle(collider)
        .insert(ColliderDebugRender::default())
        .insert(ColliderPositionSync::Discrete);

    /*
     * Create the cubes
     */
    let num = 8;
    let rad = 1.0;

    let shift = rad * 2.0 + rad;
    let centerx = shift * (num / 2) as f32;
    let centery = shift / 2.0;
    let centerz = shift * (num / 2) as f32;

    let mut offset = -(num as f32) * (rad * 2.0 + rad) * 0.5;
    let mut color = 0;

    for j in 0usize..20 {
        for i in 0..num {
            for k in 0usize..num {
                let x = i as f32 * shift - centerx + offset;
                let y = j as f32 * shift + centery + 3.0;
                let z = k as f32 * shift - centerz + offset;
                color += 1;

                // Build the rigid body.
                let rigid_body = RigidBodyBundle {
                    position: [x, y, z].into(),
                    ..RigidBodyBundle::default()
                };

                let collider = ColliderBundle {
                    shape: ColliderShape::cuboid(rad, rad, rad).into(),
                    ..ColliderBundle::default()
                };

                commands
                    .spawn()
                    .insert_bundle(rigid_body)
                    .insert_bundle(collider)
                    .insert(ColliderDebugRender::with_id(color))
                    .insert(ColliderPositionSync::Discrete);
            }
        }

        offset -= 0.05 * rad * (num as f32 - 1.0);
    }
}

fn cast_ray(
    mut commands: Commands,
    mut materials: ResMut<Assets<StandardMaterial>>,
    windows: Res<Windows>,
    query_pipeline: Res<QueryPipeline>,
    colliders: QueryPipelineColliderComponentsQuery,
    bodies: Query<&RigidBodyTypeComponent>,
    cameras: Query<(&Camera, &GlobalTransform)>,
) {
    // We will color in read the colliders hovered by the mouse.
    for (camera, camera_transform) in cameras.iter() {
        // First, compute a ray from the mouse position.
        let ray = ray_from_mouse_position(windows.get_primary().unwrap(), camera, camera_transform);

        // Then cast the ray. This QueryPipelineColliderComponentsSet is just
        // an adapter so the query pipeline understands how to use the query from Bevy.
        let colliders = QueryPipelineColliderComponentsSet(&colliders);
        let hit = query_pipeline.cast_ray(
            &colliders,
            &ray,
            100.0,
            true,
            InteractionGroups::default(),
            None,
        );

        if let Some(hit) = hit {
            // Color in red the entity we just hit.
            // But don't color it if the rigid-body is not dynamic.
            if let Ok(rb_type) = bodies.get(hit.0.entity()) {
                if rb_type.0 == RigidBodyType::Dynamic {
                    // TODO: don't create a new material every time.
                    let material = materials.add(Color::rgb(1.0, 0.0, 0.0).into());
                    commands.entity(hit.0.entity()).insert(material);
                }
            }
        }
    }
}

// Credit to @doomy on discord.
fn ray_from_mouse_position(
    window: &Window,
    camera: &Camera,
    camera_transform: &GlobalTransform,
) -> Ray {
    let mouse_position = window.cursor_position().unwrap_or(Vec2::new(0.0, 0.0));

    let x = 2.0 * (mouse_position.x / window.width() as f32) - 1.0;
    let y = 2.0 * (mouse_position.y / window.height() as f32) - 1.0;

    let camera_inverse_matrix =
        camera_transform.compute_matrix() * camera.projection_matrix.inverse();
    let near = camera_inverse_matrix * Vec3::new(x, y, 0.0).extend(1.0);
    let far = camera_inverse_matrix * Vec3::new(x, y, 1.0).extend(1.0);

    let near = near.truncate() / near.w;
    let far = far.truncate() / far.w;
    let dir: Vec3 = far - near;

    Ray::new(near.into(), dir.into())
}
