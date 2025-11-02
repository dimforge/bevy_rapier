//! Module for utilities to convert from a [`rapier::prelude::Shape`] to a [`bevy::prelude::Mesh`].

use super::Collider;
use crate::rapier::prelude::Ball;
#[cfg(feature = "dim3")]
use crate::rapier::prelude::{Cone, Cylinder};
#[cfg(feature = "dim2")]
use bevy::mesh::{Capsule2dMeshBuilder, CircleMeshBuilder};
#[cfg(feature = "dim3")]
use bevy::mesh::{
    Capsule3dMeshBuilder, ConeMeshBuilder, CylinderMeshBuilder, PlaneMeshBuilder, SphereMeshBuilder,
};
use bevy::{
    asset::RenderAssetUsages,
    mesh::Indices,
    prelude::{Mesh, MeshBuilder},
};

use rapier::prelude::{Capsule, TypedShape};
/// Converts a [`TypedShape`] to a [`Mesh`].
///
/// This expects a [`TypedShape`] to be a convertible to a bavy builtin [`bevy::prelude::Meshable`],
/// Or builds a new Mesh with [`PrimitiveTopology::TriangleList`](bevy::mesh::PrimitiveTopology::TriangleList).
pub fn typed_shape_to_mesh(typed_shape: &TypedShape) -> Option<Mesh> {
    Some(match typed_shape {
        TypedShape::Ball(ball) => ball.mesh_builder().build(),
        TypedShape::Cuboid(cuboid) => {
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
        TypedShape::Capsule(capsule) => capsule.mesh_builder().build(),
        TypedShape::Segment(_segment) => {
            // FIXME: use a LineStrip
            log::warn!("Segment not implemented yet");
            return None;
        }
        TypedShape::Triangle(triangle) => {
            // FIXME: bevy 0.16 will expose a builder for triangles: https://github.com/bevyengine/bevy/pull/17454
            let a = triangle.a.coords;
            let b = triangle.b.coords;
            let c = triangle.c.coords;
            #[cfg(feature = "dim2")]
            let mesh = bevy::prelude::Triangle3d::new(
                bevy::prelude::Vec3::new(a.x, a.y, 0.0),
                bevy::prelude::Vec3::new(b.x, b.y, 0.0),
                bevy::prelude::Vec3::new(c.x, c.y, 0.0),
            );
            #[cfg(feature = "dim3")]
            let mesh = bevy::prelude::Triangle3d::new(a.into(), b.into(), c.into());

            mesh.into()
        }
        TypedShape::Voxels(_voxels) => {
            #[cfg(feature = "dim3")]
            {
                let (vtx, idx) = _voxels.to_outline();

                let mesh = Mesh::new(
                    bevy::mesh::PrimitiveTopology::TriangleList,
                    RenderAssetUsages::default(),
                )
                .with_inserted_indices(Indices::U32(idx.into_iter().flatten().collect()));
                return Some(
                    mesh.with_inserted_attribute(
                        Mesh::ATTRIBUTE_POSITION,
                        vtx.iter()
                            .map(|pos| [pos.x, pos.y, pos.z])
                            .collect::<Vec<_>>(),
                    ),
                );
            }

            #[cfg(feature = "dim2")]
            {
                log::warn!("Voxels not implemented in 2d yet");
                // let (vtx, idx) = voxels.to_polyline();
                return None;
            }
        }
        TypedShape::TriMesh(tri_mesh) => {
            let vertices = tri_mesh.vertices();
            #[cfg(feature = "dim2")]
            let vertices: Vec<_> = vertices.iter().map(|pos| [pos.x, pos.y, 0.0]).collect();
            #[cfg(feature = "dim3")]
            let vertices: Vec<_> = vertices.iter().map(|pos| [pos.x, pos.y, pos.z]).collect();
            let indices = tri_mesh.indices();
            let mesh = Mesh::new(
                bevy::mesh::PrimitiveTopology::TriangleList,
                RenderAssetUsages::default(),
            )
            .with_inserted_indices(Indices::U32(indices.iter().cloned().flatten().collect()));
            mesh.with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, vertices)
        }
        TypedShape::Polyline(_polyline) => {
            // FIXME: use a LineStrip
            log::warn!("Polyline not implemented yet");
            return None;
        }
        TypedShape::HalfSpace(_half_space) => {
            // FIXME: We can't really implement halfspace to mesh, but we can provide a builder where user provides a size.
            log::warn!(
                    "HalfSpace cannot be converted automatically to a mesh, use [`ToMeshBuilder`] instead"
                );
            return None;
        }
        TypedShape::HeightField(_height_field) => {
            #[cfg(feature = "dim2")]
            {
                // FIXME: use a LineStrip, or a triangle mesh with a height through a builder?
                // Investigate if we can extrude a linestrip, in that case the builder wouldn't be needed.
                log::warn!("HeightField for 2d not implemented yet");
                return None;
            }
            #[cfg(feature = "dim3")]
            {
                // FIXME: we could use TriMesh::From(height_field), but that would clone, we should fix that in parry.
                let (vtx, idx) = _height_field.to_trimesh();

                let mesh = Mesh::new(
                    bevy::mesh::PrimitiveTopology::TriangleList,
                    RenderAssetUsages::default(),
                )
                .with_inserted_indices(Indices::U32(idx.into_iter().flatten().collect()));
                mesh.with_inserted_attribute(
                    Mesh::ATTRIBUTE_POSITION,
                    vtx.iter()
                        .map(|pos| [pos.x, pos.y, pos.z])
                        .collect::<Vec<_>>(),
                )
            }
        }
        TypedShape::Compound(compound) => {
            let mut final_mesh: Option<Mesh> = Option::None;
            for shape in compound.shapes() {
                let typed_shape = shape.1.as_typed_shape();
                let mesh = typed_shape_to_mesh(&typed_shape)?;
                if let Some(ref mut final_mesh) = final_mesh {
                    // FIXME: check the result when released upstream (https://github.com/bevyengine/bevy/pull/17475)
                    // FIXME: Error is simply "peeked" and ignored. Handle it properly by returning a Result from this function.
                    let _ = final_mesh.merge(&mesh).inspect_err(|e| {
                        log::warn!("Error merging mesh data: {e}");
                    });
                } else {
                    final_mesh = Some(mesh);
                }
            }
            final_mesh?
        }
        #[cfg(feature = "dim2")]
        TypedShape::ConvexPolygon(convex_polygon) => {
            let vertices = convex_polygon.points();
            let vertices: Vec<_> = vertices.iter().map(|pos| [pos.x, pos.y, 0.0]).collect();

            let indices = (1..vertices.len() as u32 - 1)
                .flat_map(|i| vec![0, i, i + 1])
                .collect::<Vec<u32>>();
            let mesh = Mesh::new(
                bevy::mesh::PrimitiveTopology::TriangleList,
                RenderAssetUsages::default(),
            )
            .with_inserted_indices(Indices::U32(indices));
            mesh.with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, vertices)
        }
        #[cfg(feature = "dim3")]
        TypedShape::ConvexPolyhedron(convex_polyhedron) => {
            let vertices = convex_polyhedron.points();
            let vertices: Vec<_> = vertices.iter().map(|pos| [pos.x, pos.y, pos.z]).collect();

            let indices = (1..vertices.len() as u32 - 1)
                .flat_map(|i| vec![0, i, i + 1])
                .collect::<Vec<u32>>();
            let mesh = Mesh::new(
                bevy::mesh::PrimitiveTopology::TriangleList,
                RenderAssetUsages::default(),
            )
            .with_inserted_indices(Indices::U32(indices));
            mesh.with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, vertices)
        }
        #[cfg(feature = "dim3")]
        TypedShape::Cone(cone) => cone.mesh_builder().build(),
        #[cfg(feature = "dim3")]
        TypedShape::Cylinder(cylinder) => cylinder.mesh_builder().build(),
        #[cfg(feature = "dim3")]
        TypedShape::RoundCone(_round_cone) => {
            // FIXME: parry doesn't have easy to use functions to convert RoundShapes to a mesh.
            log::warn!("RoundCone not implemented yet");
            return None;
        }
        #[cfg(feature = "dim3")]
        TypedShape::RoundCylinder(_round_cylinder) => {
            // FIXME: parry doesn't have easy to use functions to convert RoundShapes to a mesh.
            log::warn!("RoundCylinder not implemented yet");
            return None;
        }
        #[cfg(feature = "dim2")]
        TypedShape::RoundConvexPolygon(_round_shape) => {
            // FIXME: parry doesn't have easy to use functions to convert RoundShapes to a mesh.
            log::warn!("RoundConvexPolygon not implemented yet");
            return None;
        }
        #[cfg(feature = "dim3")]
        TypedShape::RoundConvexPolyhedron(_round_shape) => {
            // FIXME: parry doesn't have easy to use functions to convert RoundShapes to a mesh.
            log::warn!("RoundConvexPolyhedron not implemented yet");
            return None;
        }
        TypedShape::RoundCuboid(_round_shape) => {
            // FIXME: parry doesn't have easy to use functions to convert RoundShapes to a mesh.
            log::warn!("RoundCuboid not implemented yet");
            return None;
        }
        TypedShape::RoundTriangle(_round_shape) => {
            // FIXME: parry doesn't have easy to use functions to convert RoundShapes to a mesh.
            log::warn!("RoundTriangle not implemented yet");
            return None;
        }
        TypedShape::Custom(_shape) => {
            // FIXME: I'm not sure how to convert a custom shape to a mesh.
            log::warn!("Custom shape not implemented yet");
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

/// Trait to convert a parry shape to a [`MeshBuilder`].
pub trait ToMeshBuilder {
    /// Specific [`MeshBuilder`] being returned.
    type MeshBuilder: bevy::mesh::prelude::MeshBuilder;
    /// Returns a dedicated [`MeshBuilder`].
    fn mesh_builder(&self) -> Self::MeshBuilder;
}

#[cfg(feature = "dim2")]
impl ToMeshBuilder for &Ball {
    type MeshBuilder = CircleMeshBuilder;

    fn mesh_builder(&self) -> Self::MeshBuilder {
        CircleMeshBuilder::new(self.radius, 16)
    }
}

#[cfg(feature = "dim3")]
impl ToMeshBuilder for &Ball {
    type MeshBuilder = SphereMeshBuilder;

    fn mesh_builder(&self) -> Self::MeshBuilder {
        SphereMeshBuilder::new(self.radius, bevy::mesh::SphereKind::Ico { subdivisions: 1 })
    }
}

#[cfg(feature = "dim3")]
impl ToMeshBuilder for &rapier3d::prelude::HalfSpace {
    type MeshBuilder = PlaneMeshBuilder;

    fn mesh_builder(&self) -> Self::MeshBuilder {
        PlaneMeshBuilder::new(
            bevy::prelude::Dir3::new(self.normal.into()).unwrap_or(bevy::prelude::Dir3::Y),
            bevy::prelude::Vec2::ONE,
        )
    }
}

#[cfg(feature = "dim2")]
impl ToMeshBuilder for &Capsule {
    type MeshBuilder = Capsule2dMeshBuilder;

    fn mesh_builder(&self) -> Self::MeshBuilder {
        bevy::mesh::Capsule2dMeshBuilder::new(self.radius, self.height(), 10)
    }
}

#[cfg(feature = "dim3")]
impl ToMeshBuilder for &Capsule {
    type MeshBuilder = Capsule3dMeshBuilder;

    fn mesh_builder(&self) -> Self::MeshBuilder {
        bevy::mesh::Capsule3dMeshBuilder::new(self.radius, self.height(), 10, 10)
    }
}
#[cfg(feature = "dim3")]
impl ToMeshBuilder for &Cone {
    type MeshBuilder = ConeMeshBuilder;

    fn mesh_builder(&self) -> Self::MeshBuilder {
        bevy::mesh::ConeMeshBuilder::new(self.radius, self.half_height * 2.0, 16)
    }
}

#[cfg(feature = "dim3")]
impl ToMeshBuilder for &Cylinder {
    type MeshBuilder = CylinderMeshBuilder;

    fn mesh_builder(&self) -> Self::MeshBuilder {
        bevy::mesh::CylinderMeshBuilder::new(self.radius, self.half_height * 2.0, 16)
    }
}
