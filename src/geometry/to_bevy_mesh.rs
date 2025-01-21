//! Module for utilities to convert from a [`rapier::prelude::Shape`] to a [`bevy::prelude::Mesh`].

use super::Collider;
use crate::rapier::prelude::{Ball, Cuboid};
use bevy::{
    asset::RenderAssetUsages,
    prelude::{Mesh, MeshBuilder, Segment3d},
    render::{
        mesh::{Indices, SphereMeshBuilder},
        prelude::*,
    },
};
use rapier::prelude::{Shape, TriMesh, TypedShape};
/// Converts a [`TypedShape`] to a [`Mesh`].
///
/// This expects a [`TypedShape`] to be a convertible to a bavy builtin [`bevy::prelude::Meshable`],
/// Or builds a new Mesh with [`PrimitiveTopology::TriangleList`](bevy::render::mesh::PrimitiveTopology::TriangleList).
pub fn typed_shape_to_mesh(typed_shape: &TypedShape) -> Option<Mesh> {
    Some(match typed_shape {
        rapier::prelude::TypedShape::Ball(ball) => ball.mesh_builder().build(),
        rapier::prelude::TypedShape::Cuboid(cuboid) => {
            // FIXME: bevy 0.16 will expose a builder for cuboids: https://github.com/bevyengine/bevy/pull/17454
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
        rapier::prelude::TypedShape::Capsule(capsule) => capsule.mesh_builder().build(),
        rapier::prelude::TypedShape::Segment(segment) => {
            // FIXME: Segment shape not implemented yet, how to represent it? A LineStrip?
            return None;
        }
        rapier::prelude::TypedShape::Triangle(triangle) => {
            // FIXME: bevy 0.16 will expose a builder for triangles: https://github.com/bevyengine/bevy/pull/17454
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
            // FIXME: Polyline shape not implemented yet, how to represent it ? BoxedPolyline3d is only a primitive.
            return None;
        }
        rapier::prelude::TypedShape::HalfSpace(half_space) => {
            // FIXME: HalfSpace shape not implemented yet, how to represent it ? its infinite property makes it difficult.
            return None;
        }
        rapier::prelude::TypedShape::HeightField(height_field) => {
            #[cfg(feature = "dim2")]
            // FIXME: "HeightField for 2d not implemented yet, how to represent it ? its effectively a line.
            return None;
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
            let mut final_mesh = Option::None;
            for shape in compound.shapes() {
                let typed_shape = shape.1.as_typed_shape();
                let mesh = typed_shape_to_mesh(&typed_shape)?;
                if let Some(mesh) = final_mesh {
                    final_mesh = Some(mesh.combine(&mesh));
                } else {
                    final_mesh = Some(mesh);
                }
            }
            final_mesh
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
        rapier::prelude::TypedShape::Cone(cone) => mesh.mesh_builder().build(),
        #[cfg(feature = "dim3")]
        rapier::prelude::TypedShape::Cylinder(cylinder) => mesh.mesh_builder().build(),
        #[cfg(feature = "dim3")]
        rapier::prelude::TypedShape::RoundCone(round_cone) => {
            // FIXME: parry doesn't have easy to use functions to convert RoundShapes to a mesh.
            return None;
        }
        #[cfg(feature = "dim3")]
        rapier::prelude::TypedShape::RoundCylinder(round_cylinder) => {
            // FIXME: parry doesn't have easy to use functions to convert RoundShapes to a mesh.
            return None;
        }
        #[cfg(feature = "dim2")]
        rapier::prelude::TypedShape::RoundConvexPolygon(round_shape) => {
            // FIXME: parry doesn't have easy to use functions to convert RoundShapes to a mesh.
            return None;
        }
        #[cfg(feature = "dim3")]
        rapier::prelude::TypedShape::RoundConvexPolyhedron(round_shape) => {
            // FIXME: parry doesn't have easy to use functions to convert RoundShapes to a mesh.
            return None;
        }
        rapier::prelude::TypedShape::RoundCuboid(round_shape) => {
            // FIXME: parry doesn't have easy to use functions to convert RoundShapes to a mesh.
            return None;
        }
        rapier::prelude::TypedShape::RoundTriangle(round_shape) => {
            // FIXME: parry doesn't have easy to use functions to convert RoundShapes to a mesh.
            return None;
        }
        rapier::prelude::TypedShape::Custom(shape) => {
            // FIXME: I'm not sure how to convert a custom shape to a mesh.
            return None;
        }
    })
}

impl TryFrom<&Collider> for Mesh {
    type Error = ();

    fn try_from(collider: &Collider) -> Result<Self, Self::Error> {
        let typed_shape = collider.raw.as_typed_shape();
        typed_shape_to_mesh(&typed_shape).ok_or(())
    }
}

pub trait ToMeshBuilder {
    type MeshBuilder: MeshBuilder;
    fn mesh_builder(&self) -> Self::MeshBuilder;
}

#[cfg(feature = "dim2")]
impl ToMeshBuilder for Ball {
    type MeshBuilder = CircleMeshBuilder;

    fn mesh_builder(&self) -> Self::MeshBuilder {
        CircleMeshBuilder::new(self.radius, 16)
    }
}

#[cfg(feature = "dim3")]
impl ToMeshBuilder for Ball {
    type MeshBuilder = SphereMeshBuilder;

    fn mesh_builder(&self) -> Self::MeshBuilder {
        SphereMeshBuilder::new(
            self.radius,
            bevy::render::mesh::SphereKind::Ico { subdivisions: 1 },
        )
    }
}

#[cfg(feature = "dim2")]
impl ToMeshBuilder for Capsule {
    type MeshBuilder = Capsule2dMeshBuilder;

    fn mesh_builder(&self) -> Self::MeshBuilder {
        bevy::render::mesh::Capsule2dMeshBuilder::new(radius, half_height * 2.0, 10)
    }
}

#[cfg(feature = "dim3")]
impl ToMeshBuilder for Capsule {
    type MeshBuilder = Capsule3dMeshBuilder;

    fn mesh_builder(&self) -> Self::MeshBuilder {
        bevy::render::mesh::Capsule3dMeshBuilder::new(self.radius, self.half_height * 2.0, 10, 10)
    }
}
#[cfg(feature = "dim3")]
impl ToMeshBuilder for Cone {
    type MeshBuilder = ConeMeshBuilder;

    fn mesh_builder(&self) -> Self::MeshBuilder {
        bevy::render::mesh::ConeMeshBuilder::new(self.radius, self.half_height * 2.0, 16)
    }
}

#[cfg(feature = "dim3")]
impl ToMeshBuilder for Cylinder {
    type MeshBuilder = CylinderMeshBuilder;

    fn mesh_builder(&self) -> Self::MeshBuilder {
        bevy::render::mesh::CylinderMeshBuilder::new(self.radius, self.half_height * 2.0, 16)
    }
}
