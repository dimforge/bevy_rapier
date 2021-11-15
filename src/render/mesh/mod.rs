mod cube;
pub use self::cube::wire_cube;

mod sphere;
pub use self::sphere::wire_sphere;

mod trimesh;
pub use self::trimesh::wire_trimesh;

mod capsule;
pub use self::capsule::wire_capsule;

mod polyline;
pub use self::polyline::wire_polyline;

mod segment;
pub use self::segment::wire_segment;

#[cfg(feature = "dim3")]
mod cylinder;
#[cfg(feature = "dim3")]
pub use self::cylinder::wire_cylinder;

use bevy::prelude::*;
use bevy::render::pipeline::PrimitiveTopology;
use bevy::render::mesh::Indices;
use crate::prelude::*;

#[cfg(feature = "dim3")]
pub fn position_mesh(debug: &crate::render::entities::RapierDebugPosition, _config: &RapierConfiguration) -> bevy::prelude::Mesh {
        let half_size = debug.size/2.0;
        let mut mesh = Mesh::new(PrimitiveTopology::LineList);
        mesh.set_attribute(
            Mesh::ATTRIBUTE_POSITION,
            vec![
                // X
                [half_size, 0.0, 0.0],
                [-half_size, 0.0, 0.0],
                // Y
                [0.0, -half_size, 0.0],
                [0.0, half_size, 0.0],
                // Z
                [0.0, 0.0, half_size],
                [0.0, 0.0, -half_size],
            ],
        );
        mesh.set_attribute(Mesh::ATTRIBUTE_COLOR, vec![
            debug.x.as_rgba_f32(),
            debug.x.as_rgba_f32(),
            debug.y.as_rgba_f32(),
            debug.y.as_rgba_f32(),
            debug.z.as_rgba_f32(),
            debug.z.as_rgba_f32()
        ]);
        mesh.set_indices(Some(Indices::U16(vec![0, 1, 2, 3, 4, 5])));
        mesh
}

#[cfg(feature = "dim2")]
pub fn position_mesh(debug: &super::entities::RapierDebugPosition, config: &RapierConfiguration) -> bevy::prelude::Mesh {
        let half_size = debug.size/2.0;
        let mut mesh = Mesh::new(PrimitiveTopology::LineList);
        mesh.set_attribute(
            Mesh::ATTRIBUTE_POSITION,
            vec![
                // X
                [half_size * config.scale, 0.0, 0.0],
                [-(half_size * config.scale), 0.0, 0.0],
                // Y
                [0.0, -(half_size * config.scale), 0.0],
                [0.0, half_size * config.scale, 0.0],
            ],
        );
        mesh.set_attribute(Mesh::ATTRIBUTE_COLOR, vec![
            debug.x.as_rgba_f32(),
            debug.x.as_rgba_f32(),
            debug.y.as_rgba_f32(),
            debug.y.as_rgba_f32()
        ]);
        mesh.set_indices(Some(Indices::U16(vec![0, 1, 2, 3])));
        mesh
}
