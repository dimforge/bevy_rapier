use std::{
    convert::{TryFrom, TryInto},
    error::Error,
    fmt, usize,
};

use crate::{na::Point, prelude::Real};
use bevy::prelude::Mesh;
use bevy::render::pipeline::VertexFormat::Float3;
use rapier::math::DIM;

#[derive(Debug, Clone, Default)]
pub struct VertexFormatError();

impl fmt::Display for VertexFormatError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "invalid vertex buffer format! Only Float3 is allowed")
    }
}

impl Error for VertexFormatError {}

#[derive(Debug, Clone, Default)]
pub struct VertexBufferLayoutMissing();

impl fmt::Display for VertexBufferLayoutMissing {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "vertex buffer layout is missing")
    }
}

impl Error for VertexBufferLayoutMissing {}

#[derive(Debug, Clone, Default)]
pub struct OffsetTooBig();

impl fmt::Display for OffsetTooBig {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "vertex position attribute offset is too big")
    }
}

impl Error for OffsetTooBig {}

#[derive(Debug, Clone, Default)]
pub struct BufferChunkTooBig();

impl fmt::Display for BufferChunkTooBig {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "Buffer chunk is too big")
    }
}

impl Error for BufferChunkTooBig {}

#[derive(Debug, Clone, Default)]
pub struct BufferDataFormatError();

impl fmt::Display for BufferDataFormatError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "Buffer data format does not match layout")
    }
}

impl Error for BufferDataFormatError {}

#[derive(Debug, Clone)]
pub enum ErrorSum {
    VertexBufferLayoutMissing(VertexBufferLayoutMissing),
    VertexFormatError(VertexFormatError),
    OffsetTooBig(OffsetTooBig),
    BufferChunkTooBig(BufferChunkTooBig),
    BufferDataFormatError(BufferDataFormatError)
}
impl Error for ErrorSum {}

impl fmt::Display for ErrorSum {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match &self {
            ErrorSum::VertexBufferLayoutMissing(e) => e.fmt(f),
            ErrorSum::VertexFormatError(e) => e.fmt(f),
            ErrorSum::OffsetTooBig(e) => e.fmt(f),
            ErrorSum::BufferChunkTooBig(e) => e.fmt(f),
            ErrorSum::BufferDataFormatError(e) => e.fmt(f),
        }
    }
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
            .ok_or(ErrorSum::VertexBufferLayoutMissing(Default::default()))?;

        let vert_pos_start: usize = (vert_pos_attr.offset)
            .try_into()
            .or_else(|_| Err(ErrorSum::OffsetTooBig(Default::default())))?;
        let vertex_chunk_bytes: usize = vert_buffer_layout
            .attributes
            .iter()
            .map(|x| x.format.get_size())
            .sum::<u64>()
            .try_into()
            .or_else(|_| Err(ErrorSum::BufferChunkTooBig(Default::default())))?;

        let vert_pos_end: usize = match vert_pos_attr.format {
            Float3 => {
                let pos_size: usize = Float3
                    .get_size()
                    .try_into()
                    .or_else(|_| Err(ErrorSum::BufferChunkTooBig(Default::default())))?;
                Ok(vert_pos_start + pos_size)
            }
            _ => Err(ErrorSum::VertexFormatError(Default::default())),
        }?;

        let coll_verts: Vec<Point<f32, DIM>> = mesh
            .0
            .get_vertex_buffer_data()
            .chunks(vertex_chunk_bytes)
            .map(|x| {
                let pos_vec= x[vert_pos_start..vert_pos_end]
                    .chunks(4)
                    .map(|x| Ok(f32::from_ne_bytes(x.try_into()?)))
                    .collect::<Result<Vec<f32>, Box<dyn Error>>>();
                match pos_vec {
                    Ok(pos_vec) => match pos_vec.len() {
                       DIM => Ok(Point::from_slice(pos_vec.as_slice())),
                       _ => Err(ErrorSum::BufferDataFormatError(Default::default())),
                    },
                    Err(_) => Err(ErrorSum::BufferDataFormatError(Default::default())),
                }
            })
            .collect::<Result<Vec<Point<f32, DIM>>, ErrorSum>>()?;

        let coll_indices: Vec<u32> = mesh
            .0
            .get_index_buffer_bytes()
            .unwrap()
            .chunks(4)
            .map(|x| {
                match x.try_into() {
                    Ok(x) => Ok(u32::from_ne_bytes(x)) ,
                    Err(_) => Err(ErrorSum::BufferDataFormatError(Default::default())),
                }
            })
            .collect::<Result<Vec<u32>, ErrorSum>>()?;

        let coll_indices_chunked: Vec<[u32; DIM]> = coll_indices
            .chunks(3)
            .map(|x| {
                match x.try_into() {
                    Ok(x) => Ok(x),
                    Err(_) => Err(ErrorSum::BufferDataFormatError(Default::default())),
                }
            })
            .collect::<Result<Vec<[u32; DIM]>, ErrorSum>>()?;

        Ok((coll_verts, coll_indices_chunked))
    }

    type Error = ErrorSum;
}
