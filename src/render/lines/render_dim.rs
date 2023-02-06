#[cfg(feature = "debug-render-3d")]
pub mod r3d {
    use bevy::{
        core_pipeline::core_3d::Opaque3d,
        pbr::{
            DrawMesh, MeshPipeline, MeshPipelineKey, MeshUniform, SetMeshBindGroup,
            SetMeshViewBindGroup, MAX_CASCADES_PER_LIGHT, MAX_DIRECTIONAL_LIGHTS,
        },
        prelude::*,
        render::{
            mesh::MeshVertexBufferLayout,
            render_asset::RenderAssets,
            render_phase::{DrawFunctions, RenderPhase, SetItemPipeline},
            render_resource::{
                BlendComponent, BlendFactor, BlendOperation, BlendState, ColorTargetState,
                ColorWrites, CompareFunction, DepthBiasState, DepthStencilState, FragmentState,
                FrontFace, MultisampleState, PipelineCache, PolygonMode, PrimitiveState,
                PrimitiveTopology, RenderPipelineDescriptor, ShaderDefVal, SpecializedMeshPipeline,
                SpecializedMeshPipelineError, SpecializedMeshPipelines, StencilFaceState,
                StencilState, TextureFormat, VertexState,
            },
            texture::BevyDefault,
            view::{ExtractedView, Msaa, ViewTarget},
        },
    };

    use super::super::{DebugLinesConfig, RenderDebugLinesMesh, DEBUG_LINES_SHADER_HANDLE};

    #[derive(Resource)]
    pub(crate) struct DebugLinePipeline {
        mesh_pipeline: MeshPipeline,
        shader: Handle<Shader>,
    }
    impl FromWorld for DebugLinePipeline {
        fn from_world(render_world: &mut World) -> Self {
            DebugLinePipeline {
                mesh_pipeline: render_world.resource::<MeshPipeline>().clone(),
                shader: DEBUG_LINES_SHADER_HANDLE.typed(),
            }
        }
    }

    impl SpecializedMeshPipeline for DebugLinePipeline {
        type Key = (bool, MeshPipelineKey);

        fn specialize(
            &self,
            (depth_test, key): Self::Key,
            layout: &MeshVertexBufferLayout,
        ) -> Result<RenderPipelineDescriptor, SpecializedMeshPipelineError> {
            let mut shader_defs = Vec::new();
            shader_defs.push("LINES_3D".into());
            if depth_test {
                shader_defs.push("DEPTH_TEST_ENABLED".into());
            }

            shader_defs.push(ShaderDefVal::UInt(
                "MAX_CASCADES_PER_LIGHT".to_string(),
                MAX_CASCADES_PER_LIGHT as u32,
            ));
            shader_defs.push(ShaderDefVal::UInt(
                "MAX_DIRECTIONAL_LIGHTS".to_string(),
                MAX_DIRECTIONAL_LIGHTS as u32,
            ));

            let vertex_buffer_layout = layout.get_layout(&[
                Mesh::ATTRIBUTE_POSITION.at_shader_location(0),
                Mesh::ATTRIBUTE_COLOR.at_shader_location(1),
            ])?;

            let format = if key.contains(MeshPipelineKey::HDR) {
                ViewTarget::TEXTURE_FORMAT_HDR
            } else {
                TextureFormat::bevy_default()
            };

            let mut bind_group_layout = match key.msaa_samples() {
                1 => vec![self.mesh_pipeline.view_layout.clone()],
                _ => {
                    shader_defs.push("MULTISAMPLED".into());
                    vec![self.mesh_pipeline.view_layout_multisampled.clone()]
                }
            };

            Ok(RenderPipelineDescriptor {
                vertex: VertexState {
                    shader: self.shader.clone_weak(),
                    entry_point: "vertex".into(),
                    shader_defs: shader_defs.clone(),
                    buffers: vec![vertex_buffer_layout],
                },
                fragment: Some(FragmentState {
                    shader: self.shader.clone_weak(),
                    shader_defs,
                    entry_point: "fragment".into(),
                    targets: vec![Some(ColorTargetState {
                        format,
                        blend: Some(BlendState::ALPHA_BLENDING),
                        write_mask: ColorWrites::ALL,
                    })],
                }),
                layout: Some(bind_group_layout),
                primitive: PrimitiveState {
                    topology: PrimitiveTopology::LineList,
                    ..default()
                },
                depth_stencil: Some(DepthStencilState {
                    format: TextureFormat::Depth32Float,
                    depth_write_enabled: false,
                    depth_compare: CompareFunction::Greater,
                    stencil: default(),
                    bias: default(),
                }),
                multisample: MultisampleState {
                    count: key.msaa_samples(),
                    mask: !0,
                    alpha_to_coverage_enabled: false,
                },
                label: Some("debug_line_pipeline".into()),
            })
        }
    }

    pub(crate) fn queue(
        opaque_3d_draw_functions: Res<DrawFunctions<Opaque3d>>,
        debug_line_pipeline: Res<DebugLinePipeline>,
        mut pipelines: ResMut<SpecializedMeshPipelines<DebugLinePipeline>>,
        mut pipeline_cache: ResMut<PipelineCache>,
        render_meshes: Res<RenderAssets<Mesh>>,
        msaa: Res<Msaa>,
        material_meshes: Query<(Entity, &MeshUniform, &Handle<Mesh>), With<RenderDebugLinesMesh>>,
        config: Res<DebugLinesConfig>,
        mut views: Query<(&ExtractedView, &mut RenderPhase<Opaque3d>)>,
    ) {
        let always_on_top = *config.always_on_top.read().unwrap();
        let draw_custom = opaque_3d_draw_functions
            .read()
            .get_id::<DrawDebugLines>()
            .unwrap();
        let msaa_key = MeshPipelineKey::from_msaa_samples(msaa.samples());
        for (view, mut transparent_phase) in views.iter_mut() {
            let view_matrix = view.transform.compute_matrix();
            let view_row_2 = view_matrix.row(2);

            let view_key = msaa_key | MeshPipelineKey::from_hdr(view.hdr);

            for (entity, mesh_uniform, mesh_handle) in material_meshes.iter() {
                if let Some(mesh) = render_meshes.get(mesh_handle) {
                    let key = view_key
                        | MeshPipelineKey::from_primitive_topology(mesh.primitive_topology);

                    let pipeline = pipelines
                        .specialize(
                            &mut pipeline_cache,
                            &debug_line_pipeline,
                            (!always_on_top, key),
                            &mesh.layout,
                        )
                        .unwrap();
                    transparent_phase.add(Opaque3d {
                        entity,
                        pipeline,
                        draw_function: draw_custom,
                        distance: view_row_2.dot(mesh_uniform.transform.col(3)),
                    });
                }
            }
        }
    }

    pub(crate) type DrawDebugLines = (
        SetItemPipeline,
        SetMeshViewBindGroup<0>,
        SetMeshBindGroup<1>,
        DrawMesh,
    );
}

#[cfg(feature = "debug-render-2d")]
pub mod r2d {
    use crate::render::lines::DebugLinesConfig;
    use bevy::{
        asset::Handle,
        core_pipeline::core_2d::Transparent2d,
        prelude::*,
        render::{
            mesh::MeshVertexBufferLayout,
            render_asset::RenderAssets,
            render_phase::{DrawFunctions, RenderPhase, SetItemPipeline},
            render_resource::{
                BlendState, ColorTargetState, ColorWrites, FragmentState, FrontFace,
                MultisampleState, PipelineCache, PolygonMode, PrimitiveState, PrimitiveTopology,
                RenderPipelineDescriptor, Shader, SpecializedMeshPipeline,
                SpecializedMeshPipelineError, SpecializedMeshPipelines, TextureFormat, VertexState,
            },
            texture::BevyDefault,
            view::{ExtractedView, Msaa, ViewTarget, VisibleEntities},
        },
        sprite::{
            DrawMesh2d, Mesh2dHandle, Mesh2dPipeline, Mesh2dPipelineKey, Mesh2dUniform,
            SetMesh2dBindGroup, SetMesh2dViewBindGroup,
        },
        utils::FloatOrd,
    };

    use super::super::{RenderDebugLinesMesh, DEBUG_LINES_SHADER_HANDLE};

    #[derive(Resource)]
    pub(crate) struct DebugLinePipeline {
        mesh_pipeline: Mesh2dPipeline,
        shader: Handle<Shader>,
    }
    impl FromWorld for DebugLinePipeline {
        fn from_world(render_world: &mut World) -> Self {
            DebugLinePipeline {
                mesh_pipeline: render_world
                    .get_resource::<Mesh2dPipeline>()
                    .unwrap()
                    .clone(),
                shader: DEBUG_LINES_SHADER_HANDLE.typed(),
            }
        }
    }

    impl SpecializedMeshPipeline for DebugLinePipeline {
        type Key = Mesh2dPipelineKey;

        fn specialize(
            &self,
            key: Self::Key,
            layout: &MeshVertexBufferLayout,
        ) -> Result<RenderPipelineDescriptor, SpecializedMeshPipelineError> {
            let vertex_buffer_layout = layout.get_layout(&[
                Mesh::ATTRIBUTE_POSITION.at_shader_location(0),
                Mesh::ATTRIBUTE_COLOR.at_shader_location(1),
            ])?;

            let format = if key.contains(Mesh2dPipelineKey::HDR) {
                ViewTarget::TEXTURE_FORMAT_HDR
            } else {
                TextureFormat::bevy_default()
            };

            Ok(RenderPipelineDescriptor {
                vertex: VertexState {
                    shader: self.shader.clone_weak(),
                    entry_point: "vertex".into(),
                    shader_defs: vec![],
                    buffers: vec![vertex_buffer_layout],
                },
                fragment: Some(FragmentState {
                    shader: self.shader.clone_weak(),
                    shader_defs: vec![],
                    entry_point: "fragment".into(),
                    targets: vec![Some(ColorTargetState {
                        format,
                        blend: Some(BlendState::ALPHA_BLENDING),
                        write_mask: ColorWrites::ALL,
                    })],
                }),
                layout: Some(vec![self.mesh_pipeline.view_layout.clone()]),
                primitive: PrimitiveState {
                    front_face: FrontFace::Ccw,
                    cull_mode: None,
                    unclipped_depth: false,
                    polygon_mode: PolygonMode::Fill,
                    conservative: false,
                    topology: PrimitiveTopology::LineList,
                    strip_index_format: None,
                },
                depth_stencil: None,
                multisample: MultisampleState {
                    count: key.msaa_samples(),
                    mask: !0,
                    alpha_to_coverage_enabled: false,
                },
                label: None,
            })
        }
    }

    pub(crate) fn queue(
        config: Res<DebugLinesConfig>,
        draw2d_functions: Res<DrawFunctions<Transparent2d>>,
        debug_line_pipeline: Res<DebugLinePipeline>,
        mut pipeline_cache: ResMut<PipelineCache>,
        mut specialized_pipelines: ResMut<SpecializedMeshPipelines<DebugLinePipeline>>,
        render_meshes: Res<RenderAssets<Mesh>>,
        msaa: Res<Msaa>,
        material_meshes: Query<(&Mesh2dUniform, &Mesh2dHandle), With<RenderDebugLinesMesh>>,
        mut views: Query<(
            &VisibleEntities,
            &mut RenderPhase<Transparent2d>,
            &ExtractedView,
        )>,
    ) {
        let always_on_top = *config.always_on_top.read().unwrap();
        for (visible_entities, mut phase, view) in views.iter_mut() {
            let draw_mesh2d = draw2d_functions.read().get_id::<DrawDebugLines>().unwrap();
            let msaa_key = Mesh2dPipelineKey::from_msaa_samples(msaa.samples());

            for visible_entity in &visible_entities.entities {
                if let Ok((uniform, mesh_handle)) = material_meshes.get(*visible_entity) {
                    if let Some(mesh) = render_meshes.get(&mesh_handle.0) {
                        let mesh_key = msaa_key
                            | Mesh2dPipelineKey::from_primitive_topology(
                                PrimitiveTopology::LineList,
                            )
                            | Mesh2dPipelineKey::from_hdr(view.hdr);

                        let mesh_z = if always_on_top {
                            f32::MAX
                        } else {
                            uniform.transform.w_axis.z
                        };
                        let pipeline = specialized_pipelines
                            .specialize(
                                &mut pipeline_cache,
                                &debug_line_pipeline,
                                mesh_key,
                                &mesh.layout,
                            )
                            .unwrap();
                        phase.add(Transparent2d {
                            entity: *visible_entity,
                            draw_function: draw_mesh2d,
                            pipeline,
                            sort_key: FloatOrd(mesh_z),
                            batch_range: None,
                        });
                    }
                }
            }
        }
    }

    pub(crate) type DrawDebugLines = (
        SetItemPipeline,
        SetMesh2dViewBindGroup<0>,
        SetMesh2dBindGroup<1>,
        DrawMesh2d,
    );
}
