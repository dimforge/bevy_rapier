use crate::physics::{ColliderHandleComponent, RapierConfiguration};
use crate::render::RapierRenderColor;
use bevy::prelude::*;
#[cfg(feature = "dim2")]
use bevy::render::mesh::{Indices, VertexAttributeValues};
use rapier::dynamics::RigidBodySet;
use rapier::geometry::{ColliderSet, ShapeType};
use std::collections::HashMap;

/// System responsible for attaching a PbrComponents to each entity having a collider.
pub fn create_collider_renders_system(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    configuration: Res<RapierConfiguration>,
    bodies: Res<RigidBodySet>,
    colliders: ResMut<ColliderSet>,
    query: Query<
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
    let mut body_colors = HashMap::new();

    for (entity, collider, debug_color) in &mut query.iter() {
        if let Some(collider) = colliders.get(collider.handle()) {
            if let Some(body) = bodies.get(collider.parent()) {
                let default_color = if body.is_static() {
                    ground_color
                } else {
                    *body_colors.entry(collider.parent()).or_insert_with(|| {
                        icolor += 1;
                        palette[icolor % palette.len()]
                    })
                };

                let shape = collider.shape();

                let color = debug_color
                    .map(|c| Color::rgb(c.0, c.1, c.2))
                    .unwrap_or(default_color);

                let mesh = match shape.shape_type() {
                    #[cfg(feature = "dim3")]
                    ShapeType::Cuboid => Mesh::from(shape::Cube { size: 1.0 }),
                    #[cfg(feature = "dim2")]
                    ShapeType::Cuboid => Mesh::from(shape::Quad {
                        size: Vec2::new(2.0, 2.0),
                        flip: false,
                    }),
                    ShapeType::Ball => Mesh::from(shape::Icosphere {
                        subdivisions: 2,
                        radius: 1.0,
                    }),
                    #[cfg(feature = "dim2")]
                    ShapeType::Trimesh => {
                        let mut mesh =
                            Mesh::new(bevy::render::pipeline::PrimitiveTopology::TriangleList);
                        let trimesh = shape.as_trimesh().unwrap();
                        mesh.set_attribute(
                            Mesh::ATTRIBUTE_POSITION,
                            VertexAttributeValues::from(
                                trimesh
                                    .vertices()
                                    .iter()
                                    .map(|vertice| [vertice.x, vertice.y])
                                    .collect::<Vec<_>>(),
                            ),
                        );
                        mesh.set_indices(Some(Indices::U32(
                            trimesh
                                .indices()
                                .iter()
                                .flat_map(|triangle| triangle.iter())
                                .cloned()
                                .collect(),
                        )));
                        mesh
                    }
                    _ => unimplemented!(),
                };

                let scale = match shape.shape_type() {
                    #[cfg(feature = "dim2")]
                    ShapeType::Cuboid => {
                        let c = shape.as_cuboid().unwrap();
                        Vec3::new(c.half_extents.x, c.half_extents.y, 1.0)
                    }
                    #[cfg(feature = "dim3")]
                    ShapeType::Cuboid => {
                        let c = shape.as_cuboid().unwrap();
                        Vec3::from_slice_unaligned(c.half_extents.as_slice())
                    }
                    ShapeType::Ball => {
                        let b = shape.as_ball().unwrap();
                        Vec3::new(b.radius, b.radius, b.radius)
                    }
                    ShapeType::Trimesh => Vec3::one(),
                    _ => unimplemented!(),
                } * configuration.scale;

                let mut transform = Transform::from_scale(scale);
                crate::physics::sync_transform(
                    collider.position_wrt_parent(),
                    configuration.scale,
                    &mut transform,
                );

                let ground_pbr = PbrComponents {
                    mesh: meshes.add(mesh),
                    material: materials.add(color.into()),
                    transform,
                    ..Default::default()
                };

                commands.insert(entity, ground_pbr);
            }
        }
    }
}
