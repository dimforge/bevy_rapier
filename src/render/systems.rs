use crate::physics::{ColliderHandleComponent, RapierPhysicsScale};
use crate::render::RapierRenderColor;
use bevy::prelude::*;
use rapier::dynamics::RigidBodySet;
use rapier::geometry::{ColliderSet, Shape};

/// System responsible for attaching a PbrComponents to each entity having a collider.
pub fn create_collider_renders_system(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    scale: Res<RapierPhysicsScale>,
    bodies: Res<RigidBodySet>,
    colliders: ResMut<ColliderSet>,
    mut query: Query<
        Without<Handle<Mesh>, (Entity, &ColliderHandleComponent, Option<&RapierRenderColor>)>,
    >,
) {
    let ground_color = Color::rgb(
        0xF3 as f32 / 255.0,
        0xD9 as f32 / 255.0,
        0xB1 as f32 / 255.0,
    );

    let palette = [
        Color::rgb(
            0x98 as f32 / 255.0,
            0xC1 as f32 / 255.0,
            0xD9 as f32 / 255.0,
        ),
        Color::rgb(
            0x05 as f32 / 255.0,
            0x3C as f32 / 255.0,
            0x5E as f32 / 255.0,
        ),
        Color::rgb(
            0x1F as f32 / 255.0,
            0x7A as f32 / 255.0,
            0x8C as f32 / 255.0,
        ),
    ];

    let mut icolor = 0;
    for (entity, collider, debug_color) in &mut query.iter() {
        if let Some(collider) = colliders.get(collider.handle()) {
            if let Some(body) = bodies.get(collider.parent()) {
                let default_color = if body.is_static() {
                    ground_color
                } else {
                    icolor += 1;
                    palette[icolor % palette.len()]
                };
                let color = debug_color
                    .map(|c| Color::rgb(c.0, c.1, c.2))
                    .unwrap_or(default_color);

                let mesh = match collider.shape() {
                    #[cfg(feature = "dim3")]
                    Shape::Cuboid(_) => Mesh::from(shape::Cube { size: 1.0 }),
                    #[cfg(feature = "dim2")]
                    Shape::Cuboid(_) => Mesh::from(shape::Quad {
                        size: Vec2::new(2.0, 2.0),
                        flip: false,
                    }),
                    Shape::Ball(_) => Mesh::from(shape::Icosphere {
                        subdivisions: 2,
                        radius: 1.0,
                    }),
                    _ => unimplemented!(),
                };

                let scale = match collider.shape() {
                    #[cfg(feature = "dim2")]
                    Shape::Cuboid(c) => Vec3::new(c.half_extents.x, c.half_extents.y, 1.0),
                    #[cfg(feature = "dim3")]
                    Shape::Cuboid(c) => Vec3::from_slice_unaligned(c.half_extents.as_slice()),
                    Shape::Ball(b) => Vec3::new(b.radius, b.radius, b.radius),
                    _ => unimplemented!(),
                } * scale.0;

                // NOTE: we can't have both the Scale and NonUniformScale components.
                // However PbrComponents automatically adds a Scale component. So
                // we add each of its field manually except for Scale.
                // That's a bit messy so surely there is a better way?
                let ground_pbr = PbrComponents {
                    mesh: meshes.add(mesh),
                    material: materials.add(color.into()),
                    transform: Transform::from_non_uniform_scale(scale),
                    ..Default::default()
                };

                commands.insert(entity, ground_pbr);
            }
        }
    }
}
