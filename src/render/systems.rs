use crate::physics::wrapper::ColliderShapeComponent;
use crate::physics::RapierConfiguration;
use crate::render::ColliderDebugRender;
use bevy::prelude::*;
use bevy::render::mesh::{Indices, VertexAttributeValues};
use bevy::sprite::MaterialMesh2dBundle;
use rapier::geometry::ShapeType;

#[derive(Debug, Hash, PartialEq, Eq, Clone, SystemLabel)]
pub enum RenderSystems {
    CreateColliderRenders,
    UpdateColliderRenderMesh,
}

/// System responsible for attaching a PbrBundle to each entity having a collider.
pub fn create_collider_renders_system(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    #[cfg(feature = "dim2")] mut materials: ResMut<Assets<ColorMaterial>>,
    #[cfg(feature = "dim3")] mut materials: ResMut<Assets<StandardMaterial>>,
    configuration: Res<RapierConfiguration>,
    collider_shapes: Query<&ColliderShapeComponent>,
    render_tags: Query<(Entity, Option<&Parent>, &ColliderDebugRender), Without<Handle<Mesh>>>,
) {
    for (entity, parent, co_render) in &mut render_tags.iter() {
        let co_shape = collider_shapes
            .get(entity)
            .ok()
            .or_else(|| collider_shapes.get(**parent?).ok());

        if let Some(co_shape) = co_shape {
            if let Some((mesh, scale)) = generate_collider_mesh(co_shape) {
                let transform = Transform::from_scale(scale * configuration.scale);

                #[cfg(feature = "dim2")]
                let bundle = MaterialMesh2dBundle {
                    mesh: meshes.add(mesh).into(),
                    material: materials.add(co_render.color.into()),
                    transform,
                    ..Default::default()
                };
                #[cfg(feature = "dim3")]
                let bundle = PbrBundle {
                    mesh: meshes.add(mesh),
                    material: materials.add(co_render.color.into()),
                    transform,
                    ..Default::default()
                };

                commands.entity(entity).insert_bundle(bundle);
            }
        }
    }
}

pub fn update_collider_render_mesh(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    configuration: Res<RapierConfiguration>,
    colliders: Query<
        (Entity, &ColliderShapeComponent),
        (
            Changed<ColliderShapeComponent>,
            With<Handle<Mesh>>,
            With<ColliderDebugRender>,
        ),
    >,
) {
    // TODO: what if the renderer is on the collider's child?
    for (entity, co_shape) in colliders.iter() {
        if let Some((mesh, scale)) = generate_collider_mesh(co_shape) {
            // TODO: remove the old mesh asset?
            let mesh = meshes.add(mesh);
            let scale = Transform::from_scale(scale * configuration.scale);
            commands.entity(entity).insert_bundle((mesh, scale));
        }
    }
}

fn generate_collider_mesh(co_shape: &ColliderShapeComponent) -> Option<(Mesh, Vec3)> {
    let mesh = match co_shape.shape_type() {
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
        ShapeType::TriMesh => {
            let mut mesh =
                Mesh::new(bevy::render::render_resource::PrimitiveTopology::TriangleList);
            let trimesh = co_shape.as_trimesh().unwrap();
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
        #[cfg(feature = "dim3")]
        ShapeType::TriMesh => {
            let mut mesh =
                Mesh::new(bevy::render::render_resource::PrimitiveTopology::TriangleList);
            let trimesh = co_shape.as_trimesh().unwrap();
            mesh.set_attribute(
                Mesh::ATTRIBUTE_POSITION,
                VertexAttributeValues::from(
                    trimesh
                        .vertices()
                        .iter()
                        .map(|vertex| [vertex.x, vertex.y, vertex.z])
                        .collect::<Vec<_>>(),
                ),
            );
            // Compute vertex normals by averaging the normals
            // of every triangle they appear in.
            // NOTE: This is a bit shonky, but good enough for visualisation.
            let verts = trimesh.vertices();
            let mut normals: Vec<Vec3> = vec![Vec3::ZERO; trimesh.vertices().len()];
            for triangle in trimesh.indices().iter() {
                let ab = verts[triangle[1] as usize] - verts[triangle[0] as usize];
                let ac = verts[triangle[2] as usize] - verts[triangle[0] as usize];
                let normal = ab.cross(&ac);
                // Contribute this normal to each vertex in the triangle.
                for i in 0..3 {
                    normals[triangle[i] as usize] += Vec3::new(normal.x, normal.y, normal.z);
                }
            }
            let normals: Vec<[f32; 3]> = normals
                .iter()
                .map(|normal| {
                    let normal = normal.normalize();
                    [normal.x, normal.y, normal.z]
                })
                .collect();
            mesh.set_attribute(Mesh::ATTRIBUTE_NORMAL, VertexAttributeValues::from(normals));
            // There's nothing particularly meaningful we can do
            // for this one without knowing anything about the overall topology.
            mesh.set_attribute(
                Mesh::ATTRIBUTE_UV_0,
                VertexAttributeValues::from(
                    trimesh
                        .vertices()
                        .iter()
                        .map(|_vertex| [0.0, 0.0])
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
        _ => return None,
    };

    let scale = match co_shape.shape_type() {
        #[cfg(feature = "dim2")]
        ShapeType::Cuboid => {
            let c = co_shape.as_cuboid().unwrap();
            Vec3::new(c.half_extents.x, c.half_extents.y, 1.0)
        }
        #[cfg(feature = "dim3")]
        ShapeType::Cuboid => {
            let c = co_shape.as_cuboid().unwrap();
            Vec3::new(c.half_extents.x, c.half_extents.y, c.half_extents.z)
        }
        ShapeType::Ball => {
            let b = co_shape.as_ball().unwrap();
            Vec3::new(b.radius, b.radius, b.radius)
        }
        ShapeType::TriMesh => Vec3::ONE,
        _ => unimplemented!(),
    };

    Some((mesh, scale))
}
