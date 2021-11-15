use bevy::prelude::*;
use bevy::render::{
    pipeline::PipelineDescriptor,
    render_graph::{base, RenderGraph, AssetRenderResourcesNode},
    shader::{ShaderStages, ShaderStage}
};
use super::{PositionWireframeMaterial, WireframeMaterial};

/// Setup the Debug Materials.
pub fn setup_material(
    mut commands: Commands,
    mut pipelines: ResMut<Assets<PipelineDescriptor>>,
    mut shaders: ResMut<Assets<Shader>>,
    mut render_graph: ResMut<RenderGraph>,
) {
    let descriptor = pipelines.set(super::COLLIDER_PIPELINE_HANDLE, PipelineDescriptor::default_config(ShaderStages {
        vertex: shaders.add(Shader::from_glsl(ShaderStage::Vertex, include_str!("shaders/wireframe.vert"))),
        fragment: Some(shaders.add(Shader::from_glsl(ShaderStage::Fragment, include_str!("shaders/wireframe.frag")))),
    }));
    commands.insert_resource(super::ColliderWireframePipelineDescriptor(descriptor));

    let descriptor = pipelines.set(super::POSITION_PIPELINE_HANDLE, PipelineDescriptor::default_config(ShaderStages {
        vertex: shaders.add(Shader::from_glsl(ShaderStage::Vertex, include_str!("shaders/position.vert"))),
        fragment: Some(shaders.add(Shader::from_glsl(ShaderStage::Fragment, include_str!("shaders/position.frag")))),
    }));
    commands.insert_resource(super::PositionPipelineDescriptor(descriptor));

    render_graph.add_system_node(
        "collider_wireframe_material",
        AssetRenderResourcesNode::<WireframeMaterial>::new(true),
    );

    render_graph.add_system_node(
        "position_wireframe_material",
        AssetRenderResourcesNode::<PositionWireframeMaterial>::new(true),
    );

    render_graph
        .add_node_edge("collider_wireframe_material", base::node::MAIN_PASS)
        .unwrap();

    render_graph
        .add_node_edge("position_wireframe_material", base::node::MAIN_PASS)
        .unwrap();
}
