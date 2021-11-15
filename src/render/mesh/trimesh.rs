use bevy::prelude::*;
use bevy::render::pipeline::PrimitiveTopology;
use bevy::render::mesh::{Indices, VertexAttributeValues};
use crate::prelude::*;

// TODO: Fix this.
#[cfg(feature = "dim3")]
pub fn wire_trimesh(trimesh: &TriMesh) -> Mesh {
    let mut mesh = Mesh::new(PrimitiveTopology::LineList);
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
    let indicies = trimesh
        .indices()
        .iter()
        .flat_map(|triangle| [triangle[0], triangle[1], triangle[2], triangle[0]])
//        .map(|x| *x)
        .collect();

    mesh.set_indices(Some(Indices::U32(indicies)));
    mesh
}

#[cfg(feature = "dim2")]
pub fn wire_trimesh(trimesh: &TriMesh) -> Mesh {
    let mut mesh = Mesh::new(PrimitiveTopology::LineList);
    mesh.set_attribute(
        Mesh::ATTRIBUTE_POSITION,
        VertexAttributeValues::from(
            trimesh
                .vertices()
                .iter()
                .map(|vertex| [vertex.x, vertex.y])
                .collect::<Vec<_>>(),
        ),
    );
    let indicies = trimesh
        .indices()
        .iter()
        .flat_map(|triangle| [triangle[0], triangle[1], triangle[0]])
//        .cloned()
        .collect();

    mesh.set_indices(Some(Indices::U32(indicies)));
    mesh
}
