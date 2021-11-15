use bevy::prelude::*;
use bevy::render::pipeline::PrimitiveTopology;
use crate::prelude::*;
use bevy::prelude::shape::CapsuleUvProfile;

pub fn wire_capsule(capsule: &Capsule, config: &RapierConfiguration) -> Mesh {
    let capsule = Mesh::from(bevy::prelude::shape::Capsule {
        radius: capsule.radius * config.scale,
        rings: 0,
        depth: capsule.half_height() / 4.0 * config.scale,
        latitudes: 8,
        longitudes: 18,
        uv_profile: CapsuleUvProfile::Aspect
    });
    let mut new_mesh = Mesh::new(PrimitiveTopology::LineList);
    new_mesh.set_attribute(Mesh::ATTRIBUTE_POSITION, capsule.attribute(Mesh::ATTRIBUTE_POSITION).unwrap().to_owned());
    new_mesh.set_indices(capsule.indices().map(|x|x.to_owned()));
    new_mesh
}
