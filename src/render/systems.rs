use crate::physics::{ColliderHandleComponent, RapierConfiguration};
use crate::render::DebugColliderShape;
use bevy::prelude::*;
#[cfg(feature = "dim2")]
use bevy::render::mesh::{Indices, VertexAttributeValues};
use rapier::dynamics::RigidBodySet;
use rapier::geometry::{ColliderSet, ShapeType};
use std::collections::HashMap;

/// System responsible for detecting changed debug components.
pub fn detect_changed_debug_components_system(
    mut debug_entities_query: Query<(&mut DebugColliderShape, Entity, Option<&Parent>)>,
    colliders_query: Query<&ColliderHandleComponent>,
) {
    for (mut debug, entity, parent) in debug_entities_query.iter_mut() {
        if let Some(collider_handle) = std::iter::once(entity)
            .chain(parent.map(|parent| parent.0).into_iter())
            .flat_map(|entity| colliders_query.get(entity).into_iter())
            .next()
        {
            let collider_handle = Some(collider_handle.handle());
            if debug.collider_handle != collider_handle {
                // Mark the component as changed.
                debug.collider_handle = collider_handle;
            }
        }
    }
}

/// System responsible for attaching a PbrBundle to each entity having a collider.
pub fn create_collider_renders_system(
    commands: &mut Commands,
    bodies: Res<RigidBodySet>,
    colliders: Res<ColliderSet>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    configuration: Res<RapierConfiguration>,
    query: Query<(Entity, &DebugColliderShape), Changed<DebugColliderShape>>,
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

    for (entity, debug) in query.iter() {
        if let Some((collider, body)) = debug.collider_handle.and_then(|collider_handle| {
            colliders
                .get(collider_handle)
                .and_then(|collider| bodies.get(collider.parent()).map(|body| (collider, body)))
        }) {
            let color = debug.color.unwrap_or_else(|| {
                if body.is_static() {
                    ground_color
                } else {
                    *body_colors.entry(collider.parent()).or_insert_with(|| {
                        icolor += 1;
                        palette[icolor % palette.len()]
                    })
                }
            });

            let shape = collider.shape();
            let mesh = match shape.shape_type() {
                #[cfg(feature = "dim3")]
                ShapeType::Cuboid => Mesh::from(shape::Cube { size: 2.0 }),
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

            let pbr = PbrBundle {
                mesh: meshes.add(mesh),
                material: materials.add(color.into()),
                transform,
                ..Default::default()
            };

            commands.insert(entity, pbr);
        }
    }
}
