use super::Collider;
use bevy::{
    asset::RenderAssetUsages,
    prelude::{Mesh, MeshBuilder, Segment3d},
    render::mesh::Indices,
};
use rapier::prelude::{Shape, TriMesh, TypedShape};

pub fn typed_shape_to_mesh(typed_shape: &TypedShape) -> Mesh {
    match typed_shape {
        rapier::prelude::TypedShape::Ball(ball) => {
            let radius = ball.radius;
            let mesh = bevy::render::mesh::SphereMeshBuilder::new(
                radius,
                bevy::render::mesh::SphereKind::Ico { subdivisions: 1 },
            );
            mesh.build()
        }
        rapier::prelude::TypedShape::Cuboid(cuboid) => {
            let half_extents = cuboid.half_extents;
            #[cfg(feature = "dim2")]
            let mesh = bevy::prelude::Rectangle::new(half_extents.x * 2.0, half_extents.y * 2.0);
            #[cfg(feature = "dim3")]
            let mesh = bevy::prelude::Cuboid::new(
                half_extents.x * 2.0,
                half_extents.y * 2.0,
                half_extents.z * 2.0,
            );
            Mesh::from(mesh)
        }
        rapier::prelude::TypedShape::Capsule(capsule) => {
            let radius = capsule.radius;
            let half_height = capsule.half_height();
            #[cfg(feature = "dim2")]
            let mesh = bevy::render::mesh::Capsule2dMeshBuilder::new(radius, half_height * 2.0, 10);
            #[cfg(feature = "dim3")]
            let mesh =
                bevy::render::mesh::Capsule3dMeshBuilder::new(radius, half_height * 2.0, 10, 10);
            mesh.build()
        }
        rapier::prelude::TypedShape::Segment(segment) => {
            todo!("Segment shape not implemented yet, how to represent it ? A LineStrip?");
        }
        rapier::prelude::TypedShape::Triangle(triangle) => {
            let a = triangle.a.coords;
            let b = triangle.b.coords;
            let c = triangle.c.coords;
            let mesh = bevy::prelude::Triangle3d::new(
                bevy::prelude::Vec3::new(a.x, a.y, 0.0),
                bevy::prelude::Vec3::new(b.x, b.y, 0.0),
                bevy::prelude::Vec3::new(c.x, c.y, 0.0),
            );
            mesh.into()
        }
        rapier::prelude::TypedShape::TriMesh(tri_mesh) => {
            let vertices = tri_mesh.vertices();
            #[cfg(feature = "dim2")]
            let vertices: Vec<_> = vertices.iter().map(|pos| [pos.x, pos.y, 0.0]).collect();
            #[cfg(feature = "dim3")]
            let vertices: Vec<_> = vertices.iter().map(|pos| [pos.x, pos.y, pos.z]).collect();
            let indices = tri_mesh.indices();
            let mesh = Mesh::new(
                bevy::render::mesh::PrimitiveTopology::TriangleList,
                RenderAssetUsages::default(),
            )
            .with_inserted_indices(Indices::U32(indices.iter().cloned().flatten().collect()));
            let mesh = mesh.with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, vertices);

            mesh.into()
        }
        rapier::prelude::TypedShape::Polyline(polyline) => {
            todo!("Polyline shape not implemented yet, how to represent it ? BoxedPolyline3d is only a primitive.");
        }
        rapier::prelude::TypedShape::HalfSpace(half_space) => {
            todo!("HalfSpace shape not implemented yet, how to represent it ? its infinite property makes it difficult.");
        }
        rapier::prelude::TypedShape::HeightField(height_field) => {
            #[cfg(feature = "dim2")]
            todo!("HeightField for 2d not implemented yet, how to represent it ? its effectively a line.");
            #[cfg(feature = "dim3")]
            {
                // FIXME: we could use TriMesh::From(height_field), but that would clone, we should fix that in parry.
                let (vtx, idx) = height_field.to_trimesh();
                let tri_mesh = TriMesh::new(vtx, idx).unwrap();

                // From Trimesh:
                let vertices = tri_mesh.vertices();
                let vertices: Vec<_> = vertices.iter().map(|pos| [pos.x, pos.y, 0.0]).collect();
                let indices = tri_mesh.indices();
                let mesh = Mesh::new(
                    bevy::render::mesh::PrimitiveTopology::TriangleList,
                    RenderAssetUsages::default(),
                )
                .with_inserted_indices(Indices::U32(indices.iter().cloned().flatten().collect()));
                let mesh = mesh.with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, vertices);

                mesh.into()
            }
        }
        rapier::prelude::TypedShape::Compound(compound) => {
            let mut vertices = Vec::new();
            let mut indices = Vec::new();
            for shape in compound.shapes() {
                let typed_shape = shape.1.as_typed_shape();
                let mesh = typed_shape_to_mesh(&typed_shape);

                assert!(mesh.primitive_topology() == bevy::render::mesh::PrimitiveTopology::TriangleList,
                "Compound shape mesh conversion does not support shapes not converting to PrimitiveTopology::TriangleList.");

                vertices.append(
                    &mut mesh
                        .attribute(Mesh::ATTRIBUTE_POSITION.id)
                        .unwrap()
                        .as_float3()
                        .unwrap()
                        .iter()
                        .cloned()
                        .flatten()
                        .collect::<Vec<_>>(),
                );
                indices.append(&mut mesh.indices().unwrap().iter().collect::<Vec<_>>());
            }
            let mesh = Mesh::new(
                bevy::render::mesh::PrimitiveTopology::TriangleList,
                RenderAssetUsages::default(),
            )
            .with_inserted_indices(Indices::U32(indices.iter().map(|i| *i as u32).collect()));
            let mesh = mesh.with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, vertices);
            mesh.into()
        }
        #[cfg(feature = "dim2")]
        rapier::prelude::TypedShape::ConvexPolygon(convex_polygon) => {
            let vertices = convex_polygon.points();
            let vertices: Vec<_> = vertices.iter().map(|pos| [pos.x, pos.y, 0.0]).collect();

            let indices = (1..vertices.len() as u32 - 1)
                .flat_map(|i| vec![0, i, i + 1])
                .collect::<Vec<u32>>();
            let mesh = Mesh::new(
                bevy::render::mesh::PrimitiveTopology::TriangleList,
                RenderAssetUsages::default(),
            )
            .with_inserted_indices(Indices::U32(indices));
            let mesh = mesh.with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, vertices);

            mesh.into()
        }
        #[cfg(feature = "dim3")]
        rapier::prelude::TypedShape::ConvexPolyhedron(convex_polyhedron) => {
            let vertices = convex_polyhedron.points();
            let vertices: Vec<_> = vertices.iter().map(|pos| [pos.x, pos.y, pos.z]).collect();

            let indices = (1..vertices.len() as u32 - 1)
                .flat_map(|i| vec![0, i, i + 1])
                .collect::<Vec<u32>>();
            let mesh = Mesh::new(
                bevy::render::mesh::PrimitiveTopology::TriangleList,
                RenderAssetUsages::default(),
            )
            .with_inserted_indices(Indices::U32(indices));
            let mesh = mesh.with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, vertices);

            mesh.into()
        }
        #[cfg(feature = "dim3")]
        rapier::prelude::TypedShape::Cone(cone) => {
            let radius = cone.radius;
            let half_height = cone.half_height;
            // TODO: implement Meshable for all TypedShape variants, that probably will have to be wrapped in a new type.
            let mesh = bevy::render::mesh::ConeMeshBuilder::new(radius, half_height * 2.0, 10);
            mesh.build()
        }
        #[cfg(feature = "dim3")]
        rapier::prelude::TypedShape::Cylinder(cylinder) => {
            let radius = cylinder.radius;
            let half_height = cylinder.half_height;
            let mesh = bevy::render::mesh::CylinderMeshBuilder::new(radius, half_height * 2.0, 10);
            mesh.build()
        }
        #[cfg(feature = "dim3")]
        rapier::prelude::TypedShape::RoundCone(round_cone) => {
            todo!("parry doesn't have easy to use functions to convert RoundShapes to a mesh.");
        }
        #[cfg(feature = "dim3")]
        rapier::prelude::TypedShape::RoundCylinder(round_cylinder) => {
            todo!("parry doesn't have easy to use functions to convert RoundShapes to a mesh.");
        }
        #[cfg(feature = "dim2")]
        rapier::prelude::TypedShape::RoundConvexPolygon(round_shape) => {
            todo!("parry doesn't have easy to use functions to convert RoundShapes to a mesh.");
        }
        #[cfg(feature = "dim3")]
        rapier::prelude::TypedShape::RoundConvexPolyhedron(round_shape) => {
            todo!("parry doesn't have easy to use functions to convert RoundShapes to a mesh.");
        }
        rapier::prelude::TypedShape::RoundCuboid(round_shape) => {
            todo!("parry doesn't have easy to use functions to convert RoundShapes to a mesh.");
        }
        rapier::prelude::TypedShape::RoundTriangle(round_shape) => {
            todo!("parry doesn't have easy to use functions to convert RoundShapes to a mesh.");
        }
        rapier::prelude::TypedShape::Custom(shape) => {
            todo!("I'm not sure how to convert a custom shape to a mesh.");
        }
    }
}

impl From<&Collider> for Mesh {
    fn from(shape: &Collider) -> Self {
        let typed_shape = shape.raw.as_typed_shape();
        typed_shape_to_mesh(&typed_shape)
    }
}
