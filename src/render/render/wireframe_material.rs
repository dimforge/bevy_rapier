use bevy::{
    reflect::TypeUuid,
    render::color::Color,
    render::renderer::RenderResources,
    render::shader::ShaderDefs
};

#[derive(RenderResources, Default, TypeUuid, ShaderDefs)]
#[uuid = "2f603f53-03f3-47c3-9e9e-e43cfcd6183e"]
pub struct WireframeMaterial {
    pub color: Color,
    #[render_resources(ignore)]
    #[shader_def]
    pub dashed: bool
}
