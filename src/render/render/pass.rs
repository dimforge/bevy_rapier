use bevy::{
    prelude::*,
    render::{
        camera::ActiveCameras,
        pass::{
            LoadOp, Operations, PassDescriptor, RenderPassDepthStencilAttachmentDescriptor,
            TextureAttachment,
        },
        render_graph::{
            base, CameraNode, PassNode, RenderGraph, RenderResourcesNode, WindowSwapChainNode,
            WindowTextureNode,
        }
    },
    ui,
};

/// Adds the custom render pass to the render graph.
pub fn setup_debug_pass(
    mut graph: ResMut<RenderGraph>,
    mut active_cameras: ResMut<ActiveCameras>,
    msaa: Res<Msaa>,
) {
    let mut debug_pass_node = PassNode::<&super::RapierDebugPass>::new(PassDescriptor {
        color_attachments: vec![msaa.color_attachment_descriptor(
            TextureAttachment::Input("color_attachment".to_string()),
            TextureAttachment::Input("color_resolve_target".to_string()),
            Operations {
                load: LoadOp::Load,
                store: true,
            },
        )],
        depth_stencil_attachment: Some(RenderPassDepthStencilAttachmentDescriptor {
            attachment: TextureAttachment::Input("depth".to_string()),
            depth_ops: Some(Operations {
                // NOTE: Clearing here makes everything in this pass be drawn on top of things in the main pass
                load: LoadOp::Clear(1.0),
                store: true,
            }),
            stencil_ops: None,
        }),
        sample_count: msaa.samples,
    });

    debug_pass_node.add_camera(super::CAMERA_RAPIER_DEBUG);
    graph.add_node(super::RAPIER_DEBUG_PASS, debug_pass_node);

    graph
        .add_slot_edge(
            base::node::PRIMARY_SWAP_CHAIN,
            WindowSwapChainNode::OUT_TEXTURE,
            super::RAPIER_DEBUG_PASS,
            if msaa.samples > 1 {
                "color_resolve_target"
            } else {
                "color_attachment"
            },
        )
        .unwrap();

    graph
        .add_slot_edge(
            base::node::MAIN_DEPTH_TEXTURE,
            WindowTextureNode::OUT_TEXTURE,
            super::RAPIER_DEBUG_PASS,
            "depth",
        )
        .unwrap();

    if msaa.samples > 1 {
        graph
            .add_slot_edge(
                base::node::MAIN_SAMPLED_COLOR_ATTACHMENT,
                WindowSwapChainNode::OUT_TEXTURE,
                super::RAPIER_DEBUG_PASS,
                "color_attachment",
            )
            .unwrap();
    }

    graph
        .add_node_edge(base::node::MAIN_PASS, super::RAPIER_DEBUG_PASS)
        .unwrap();
    graph
        .add_node_edge(super::RAPIER_DEBUG_PASS, ui::node::UI_PASS)
        .unwrap();

    graph.add_system_node(super::CAMERA_RAPIER_DEBUG, CameraNode::new(super::CAMERA_RAPIER_DEBUG));
    graph
        .add_node_edge(super::CAMERA_RAPIER_DEBUG, super::RAPIER_DEBUG_PASS)
        .unwrap();
    graph.add_system_node(super::RAPIER_DEBUG_MESH, RenderResourcesNode::<super::RapierDebugPass>::new(true));
    graph.add_node_edge(super::RAPIER_DEBUG_MESH, super::RAPIER_DEBUG_PASS).unwrap();
    graph
        .add_node_edge("collider_wireframe_material", super::RAPIER_DEBUG_PASS)
        .unwrap();

    graph
        .add_node_edge("position_wireframe_material", super::RAPIER_DEBUG_PASS)
        .unwrap();
    active_cameras.add(super::CAMERA_RAPIER_DEBUG);
}
