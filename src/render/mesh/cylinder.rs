use bevy::prelude::*;
use crate::prelude::*;

pub fn wire_cylinder(cylinder: &Cylinder, config: &RapierConfiguration) -> Mesh {
    use bevy::render::pipeline::PrimitiveTopology;
    let mut positions: Vec<[f32; 3]> = Vec::with_capacity(16 * 2);
    let mut indices: Vec<u16> = Vec::with_capacity(16 * 2 * 2 + 2 * 4);

    // Top
    for i in 0..16u16 {
        let t = (i as f32) * (2.0 / 16.0);
        let (y, x) = f32::sin_cos(t * std::f32::consts::PI);
        positions.push([x, cylinder.half_height * config.scale, y]);
    }

    // Bottom
    for i in 0..16u16 {
        let p = positions[i as usize];
        positions.push([p[0], -cylinder.half_height * config.scale, p[2]]);
    }

    for i in 0..15u16 {
        indices.push(i);
        indices.push(i + 1);
        indices.push(i + 16);
        indices.push(i + 16 + 1);
    }
    indices.push(15);
    indices.push(0);
    indices.push(15 + 16);
    indices.push(16);

    indices.push(0);
    indices.push(16);

    indices.push(4);
    indices.push(16 + 4);

    indices.push(8);
    indices.push(16 + 8);

    indices.push(12);
    indices.push(16 + 12);

    let mut mesh = Mesh::new(PrimitiveTopology::LineList);
    mesh.set_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    mesh.set_indices(Some(bevy::render::mesh::Indices::U16(indices)));
    mesh
}
