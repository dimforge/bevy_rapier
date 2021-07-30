use std::{
    convert::{TryFrom, TryInto},
    error::Error,
    fmt, usize,
};

use crate::{na::Point, prelude::Real};
use bevy::prelude::Mesh;
use rapier::math::DIM;

#[derive(Debug, Clone)]
pub struct VertexFormatError();

impl fmt::Display for VertexFormatError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "invalid vertex buffer format! Only Float3 is allowed")
    }
}

impl Error for VertexFormatError {}

#[derive(Debug, Clone)]
pub struct VertexBufferLayoutMissing();

impl fmt::Display for VertexBufferLayoutMissing {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "vertex buffer layout is missing")
    }
}

impl Error for VertexBufferLayoutMissing {}

#[derive(Debug, Clone)]
pub struct OffsetTooBig();

impl fmt::Display for OffsetTooBig {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "vertex position attribute offset is too big")
    }
}

impl Error for OffsetTooBig {}

#[derive(Debug, Clone)]
pub struct BufferChunkTooBig();

impl fmt::Display for BufferChunkTooBig {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "Buffer chunk is too big")
    }
}

impl Error for BufferChunkTooBig {}

#[derive(Debug, Clone)]
pub enum ErrorSum {
    VertexBufferLayoutMissing,
    VertexFormatError,
    OffsetTooBig,
    BufferChunkTooBig,
}

pub struct SharedShapeMesh(pub Mesh);

// Easy conversion from Bevy Mesh to types used for building SharedShape
impl TryFrom<SharedShapeMesh> for (Vec<Point<Real, DIM>>, Vec<[u32; DIM]>) {
    fn try_from(
        mesh: SharedShapeMesh,
    ) -> Result<(Vec<Point<Real, DIM>>, Vec<[u32; DIM]>), ErrorSum> {
        let vert_buffer_layout = mesh.0.get_vertex_buffer_layout();
        let vert_pos_attr = vert_buffer_layout
            .attributes
            .iter()
            .find(|&x| x.name == "Vertex_Position")
            .ok_or(ErrorSum::VertexBufferLayoutMissing)?;

        let vert_pos_start: usize = (vert_pos_attr.offset)
            .try_into()
            .or_else(|_| Err(ErrorSum::OffsetTooBig))?;
        let vertex_chunk_bytes: usize = vert_buffer_layout
            .attributes
            .iter()
            .map(|x| x.format.get_size())
            .sum::<u64>()
            .try_into()
            .or_else(|_| Err(ErrorSum::BufferChunkTooBig))?;

        let vert_pos_end = match vert_pos_attr.format {
            bevy::render::pipeline::VertexFormat::Float3 => Ok(vert_pos_start + DIM * 4),
            _ => Err(ErrorSum::VertexFormatError),
        }?;

        let coll_verts: Vec<Point<f32, DIM>> = mesh
            .0
            .get_vertex_buffer_data()
            .chunks(vertex_chunk_bytes)
            .map(|x| {
                let pos_vec: Vec<f32> = x[vert_pos_start..vert_pos_end]
                    .chunks(4)
                    .map(|x| f32::from_ne_bytes(x.try_into().expect("slice with incorrect length")))
                    .collect::<Vec<f32>>()
                    .try_into()
                    .expect("slice with incorrect length");
                Point::from_slice(pos_vec.as_slice())
            })
            .collect();

        let coll_indices: Vec<[u32; DIM]> = mesh
            .0
            .get_index_buffer_bytes()
            .unwrap()
            .chunks(4)
            .map(|x| u32::from_ne_bytes(x.try_into().expect("slice with incorrect length")))
            .collect::<Vec<u32>>()
            .chunks(DIM)
            .map(|x| x.try_into().expect("slice with incorrect length"))
            .collect();
        Ok((coll_verts, coll_indices))
    }

    type Error = ErrorSum;
}
