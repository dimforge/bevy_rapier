use bevy::{
    reflect::TypeUuid,
    render::color::Color,
    render::renderer::RenderResources
};

#[derive(RenderResources, Default, TypeUuid)]
#[uuid = "c6bbdb72-f335-48c7-9e4a-b6c5a7494f34"]
pub struct PositionWireframeMaterial {
    pub x: Color,
    pub y: Color,
    pub z: Color
}
