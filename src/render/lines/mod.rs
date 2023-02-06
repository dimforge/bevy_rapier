#![allow(warnings)]

/**
 *
 * NOTE: this module and its submodules are only temporary. It is a copy-paste of the bevy-debug-lines
 *       crate: https://github.com/Toqozz/bevy_debug_lines (MIT license)
 * It has been partially updated to work with bevy 0.7, but hasn’t been released yet.
 * So, in the mean time, we are keeping a version here that we will replace by the
 * upstream dependency once:
 * 1. The version compatible with bevy 0.7 is released to crates.io.
 * 2. We find a way to make the 2D version work with our examples. The problem
 *    only happens when running our own examples because cargo’s unification of
 *    features will enable the `3d` feature of `bevy_debug_lines` when running
 *    a `2d` example.
 *
 */
use bevy::{
    asset::{load_internal_asset, Assets, HandleUntyped},
    prelude::*,
    reflect::TypeUuid,
    render::{
        mesh::{/*Indices,*/ Mesh, VertexAttributeValues},
        render_phase::AddRenderCommand,
        render_resource::PrimitiveTopology,
        render_resource::Shader,
        view::NoFrustumCulling,
        Extract,
    },
};
use std::sync::{Arc, RwLock};

mod render_dim;

// This module exists to "isolate" the `#[cfg]` attributes to this part of the
// code. Otherwise, we would pollute the code with a lot of feature
// gates-specific code.
#[cfg(feature = "debug-render-3d")]
mod dim3 {
    pub(crate) use super::render_dim::r3d::{queue, DebugLinePipeline, DrawDebugLines};
    pub(crate) use bevy::core_pipeline::core_3d::Opaque3d as Phase;
}
#[cfg(feature = "debug-render-2d")]
mod dim2 {
    pub(crate) use super::render_dim::r2d::{queue, DebugLinePipeline, DrawDebugLines};
    pub(crate) use bevy::core_pipeline::core_2d::Transparent2d as Phase;
}

pub(crate) const SHADER_FILE: &str = include_str!("debuglines.wgsl");
pub(crate) const DEBUG_LINES_SHADER_HANDLE: HandleUntyped =
    HandleUntyped::weak_from_u64(Shader::TYPE_UUID, 17477439189930443325);

#[derive(Resource, Clone)]
pub(crate) struct DebugLinesConfig {
    pub always_on_top: Arc<RwLock<bool>>, // Don’t know how to do this properly since this resource lives in a sub-app.
}

impl DebugLinesConfig {
    fn always_on_top(on_top: bool) -> Self {
        Self {
            always_on_top: Arc::new(RwLock::new(on_top)),
        }
    }
}

/// Bevy plugin, for initializing stuff.
///
/// # Usage
///
/// ```.ignore
/// use bevy::prelude::*;
/// use bevy_prototype_debug_lines::*;
///
/// App::new()
///     .add_plugins(DefaultPlugins)
///     .add_plugin(DebugLinesPlugin::default())
///     .run();
/// ```
///
/// Alternatively, you can initialize the plugin with depth testing, so that
/// debug lines cut through geometry. To do this, use [`DebugLinesPlugin::with_depth_test(true)`].
/// ```.ignore
/// use bevy::prelude::*;
/// use bevy_prototype_debug_lines::*;
///
/// App::new()
///     .add_plugins(DefaultPlugins)
///     .add_plugin(DebugLinesPlugin::with_depth_test(true))
///     .run();
/// ```
#[derive(Debug, Default, Clone)]
pub struct DebugLinesPlugin {
    always_on_top: bool,
}

impl DebugLinesPlugin {
    /// Controls whether debug lines should be drawn with depth testing enabled
    /// or disabled.
    ///
    /// # Arguments
    ///
    /// * `val` - True if lines should intersect with other geometry, or false
    ///   if lines should always draw on top be drawn on top (the default).
    pub fn always_on_top(val: bool) -> Self {
        Self { always_on_top: val }
    }
}

#[derive(SystemSet, Hash, Debug, PartialEq, Eq, Clone)]
pub struct DrawLinesLabel;

impl Plugin for DebugLinesPlugin {
    fn build(&self, app: &mut App) {
        use bevy::render::{render_resource::SpecializedMeshPipelines, RenderApp, RenderSet};
        let mut shaders = app.world.get_resource_mut::<Assets<Shader>>().unwrap();
        shaders.set_untracked(DEBUG_LINES_SHADER_HANDLE, Shader::from_wgsl(SHADER_FILE));

        let lines_config = DebugLinesConfig::always_on_top(self.always_on_top);
        app.init_resource::<DebugLines>()
            .add_startup_system(setup)
            .add_system(
                update
                    .in_base_set(CoreSet::PostUpdate)
                    .in_set(DrawLinesLabel),
            )
            .insert_resource(lines_config.clone());

        #[cfg(feature = "debug-render-3d")]
        app.sub_app_mut(RenderApp)
            .add_render_command::<dim3::Phase, dim3::DrawDebugLines>()
            .init_resource::<dim3::DebugLinePipeline>()
            .init_resource::<SpecializedMeshPipelines<dim3::DebugLinePipeline>>()
            .add_system(dim3::queue.in_set(RenderSet::Queue));

        #[cfg(feature = "debug-render-2d")]
        app.sub_app_mut(RenderApp)
            .add_render_command::<dim2::Phase, dim2::DrawDebugLines>()
            .init_resource::<dim2::DebugLinePipeline>()
            .init_resource::<SpecializedMeshPipelines<dim2::DebugLinePipeline>>()
            .add_system(dim2::queue.in_set(RenderSet::Queue));

        app.sub_app_mut(RenderApp)
            .insert_resource(lines_config)
            .add_system_to_schedule(ExtractSchedule, extract);

        #[cfg(feature = "debug-render-3d")]
        info!("Loaded 3d debug lines plugin.");
        #[cfg(feature = "debug-render-2d")]
        info!("Loaded 2d debug lines plugin.");
    }
}

// Number of meshes to separate line buffers into.
// We don't really do culling currently but this is a gateway to that.
const MESH_COUNT: usize = 4;
// Maximum number of points for each individual mesh.
const MAX_POINTS_PER_MESH: usize = 2_usize.pow(16);
const _MAX_LINES_PER_MESH: usize = MAX_POINTS_PER_MESH / 2;
/// Maximum number of points.
pub const MAX_POINTS: usize = MAX_POINTS_PER_MESH * MESH_COUNT;
/// Maximum number of unique lines to draw at once.
pub const MAX_LINES: usize = MAX_POINTS / 2;

fn setup(mut cmds: Commands, mut meshes: ResMut<Assets<Mesh>>) {
    // Spawn a bunch of meshes to use for lines.
    for i in 0..MESH_COUNT {
        // Create a new mesh with the number of vertices we need.
        let mut mesh = Mesh::new(PrimitiveTopology::LineList);
        mesh.insert_attribute(
            Mesh::ATTRIBUTE_POSITION,
            VertexAttributeValues::Float32x3(Vec::with_capacity(MAX_POINTS_PER_MESH)),
        );
        mesh.insert_attribute(
            Mesh::ATTRIBUTE_COLOR,
            VertexAttributeValues::Float32x4(Vec::with_capacity(MAX_POINTS_PER_MESH)),
        );
        // https://github.com/Toqozz/bevy_debug_lines/issues/16
        //mesh.set_indices(Some(Indices::U16(Vec::with_capacity(MAX_POINTS_PER_MESH))));

        let mesh_handle = meshes.add(mesh);

        cmds.spawn((
            SpatialBundle::INHERITED_IDENTITY,
            NoFrustumCulling,
            DebugLinesMesh(i),
            #[cfg(feature = "debug-render-3d")]
            (
                mesh_handle.clone(),
                bevy::pbr::NotShadowCaster,
                bevy::pbr::NotShadowReceiver,
            ),
            #[cfg(feature = "debug-render-2d")]
            bevy::sprite::Mesh2dHandle(mesh_handle),
        ));
    }
}

fn update(
    #[cfg(feature = "debug-render-3d")] debug_line_meshes_3d: Query<(
        &Handle<Mesh>,
        &DebugLinesMesh,
    )>,
    #[cfg(feature = "debug-render-2d")] debug_line_meshes_2d: Query<(
        &bevy::sprite::Mesh2dHandle,
        &DebugLinesMesh,
    )>,
    time: Res<Time>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut lines: ResMut<DebugLines>,
) {
    // For each debug line mesh, fill its buffers with the relevant positions/colors chunks.
    let update = |mesh: &mut Mesh, debug_lines_idx: &DebugLinesMesh| {
        use VertexAttributeValues::{Float32x3, Float32x4};
        if let Some(Float32x3(vbuffer)) = mesh.attribute_mut(Mesh::ATTRIBUTE_POSITION) {
            vbuffer.clear();
            if let Some(new_content) = lines
                .positions
                .chunks(MAX_POINTS_PER_MESH)
                .nth(debug_lines_idx.0)
            {
                vbuffer.extend(new_content);
            }
        }

        if let Some(Float32x4(cbuffer)) = mesh.attribute_mut(Mesh::ATTRIBUTE_COLOR) {
            cbuffer.clear();
            if let Some(new_content) = lines
                .colors
                .chunks(MAX_POINTS_PER_MESH)
                .nth(debug_lines_idx.0)
            {
                cbuffer.extend(new_content);
            }
        }

        /*
        // https://github.com/Toqozz/bevy_debug_lines/issues/16
        if let Some(Indices::U16(indices)) = mesh.indices_mut() {
            indices.clear();
            if let Some(new_content) = lines.durations.chunks(_MAX_LINES_PER_MESH).nth(debug_lines_idx.0) {
                indices.extend(
                    new_content.iter().enumerate().map(|(i, _)| i as u16).flat_map(|i| [i * 2, i*2 + 1])
                );
            }
        }
        */
    };

    #[cfg(feature = "debug-render-3d")]
    for (mesh_handle, debug_lines_idx) in &debug_line_meshes_3d {
        let mesh = meshes.get_mut(mesh_handle).unwrap();
        update(mesh, debug_lines_idx);
    }

    #[cfg(feature = "debug-render-2d")]
    for (mesh2d_handle, debug_lines_idx) in &debug_line_meshes_2d {
        let mesh = meshes.get_mut(&mesh2d_handle.0).unwrap();
        update(mesh, debug_lines_idx);
    }

    // Processes stuff like getting rid of expired lines and stuff.
    lines.update(time.delta_seconds());
}

/// Move the DebugLinesMesh marker Component to the render context.
fn extract(mut commands: Commands, query: Extract<Query<Entity, With<DebugLinesMesh>>>) {
    for entity in query.iter() {
        commands.get_or_spawn(entity).insert(RenderDebugLinesMesh);
    }
}

#[derive(Component)]
pub(crate) struct DebugLinesMesh(usize);

#[derive(Component)]
pub(crate) struct RenderDebugLinesMesh;

/// Bevy resource providing facilities to draw lines.
///
/// # Usage
/// ```.ignore
/// use bevy::prelude::*;
/// use bevy_prototype_debug_lines::*;
///
/// // Draws 3 horizontal lines, which disappear after 1 frame.
/// fn some_system(mut lines: ResMut<DebugLines>) {
///     lines.line(Vec3::new(-1.0, 1.0, 0.0), Vec3::new(1.0, 1.0, 0.0), 0.0);
///     lines.line_colored(
///         Vec3::new(-1.0, 0.0, 0.0),
///         Vec3::new(1.0, 0.0, 0.0),
///         0.0,
///         Color::WHITE
///     );
///     lines.line_gradient(
///         Vec3::new(-1.0, -1.0, 0.0),
///         Vec3::new(1.0, -1.0, 0.0),
///         0.0,
///         Color::WHITE, Color::PINK
///     );
/// }
/// ```
#[derive(Resource, Default)]
pub struct DebugLines {
    pub positions: Vec<[f32; 3]>,
    pub colors: Vec<[f32; 4]>,
    pub durations: Vec<f32>,
}

impl DebugLines {
    /// Draw a line in world space, or update an existing line
    ///
    /// # Arguments
    ///
    /// * `start` - The start of the line in world space
    /// * `end` - The end of the line in world space
    /// * `duration` - Duration (in seconds) that the line should show for -- a value of
    ///   zero will show the line for 1 frame.
    pub fn line(&mut self, start: Vec3, end: Vec3, duration: f32) {
        self.line_colored(start, end, duration, Color::WHITE);
    }

    /// Draw a line in world space with a specified color, or update an existing line
    ///
    /// # Arguments
    ///
    /// * `start` - The start of the line in world space
    /// * `end` - The end of the line in world space
    /// * `duration` - Duration (in seconds) that the line should show for -- a value of
    ///   zero will show the line for 1 frame.
    /// * `color` - Line color
    pub fn line_colored(&mut self, start: Vec3, end: Vec3, duration: f32, color: Color) {
        self.line_gradient(start, end, duration, color, color);
    }

    /// Draw a line in world space with a specified gradient color, or update an existing line
    ///
    /// # Arguments
    ///
    /// * `start` - The start of the line in world space
    /// * `end` - The end of the line in world space
    /// * `duration` - Duration (in seconds) that the line should show for -- a value of
    ///   zero will show the line for 1 frame.
    /// * `start_color` - Line color
    /// * `end_color` - Line color
    pub fn line_gradient(
        &mut self,
        start: Vec3,
        end: Vec3,
        duration: f32,
        start_color: Color,
        end_color: Color,
    ) {
        if self.positions.len() >= MAX_POINTS {
            warn!("Tried to add a new line when existing number of lines was already at maximum, ignoring.");
            return;
        }

        self.positions.push(start.into());
        self.positions.push(end.into());
        self.colors.push(start_color.into());
        self.colors.push(end_color.into());
        self.durations.push(duration);
    }

    // Returns the indices of the start and end positions of the nth line.
    // The indices can also be used to access color data.
    fn nth(&self, idx: usize) -> (usize, usize) {
        let i = idx * 2;
        (i, i + 1)
    }

    // Prepare [`ImmediateLinesStorage`] and [`RetainedLinesStorage`] for next
    // frame.
    // This clears the immediate mod buffers and tells the retained mode
    // buffers to recompute expired lines list.
    fn update(&mut self, dt: f32) {
        // TODO: an actual line counter wouldn't hurt.
        let mut i = 0;
        let mut len = self.durations.len();
        while i != len {
            self.durations[i] -= dt;
            // <= instead of < is fine here because this is always called AFTER sending the
            // data to the mesh, so we're guaranteed at least a frame here.
            if self.durations[i] <= 0.0 {
                let (cur_s, cur_e) = self.nth(i);
                let (last_s, last_e) = self.nth(len - 1);
                self.positions.swap(cur_s, last_s);
                self.positions.swap(cur_e, last_e);
                self.colors.swap(cur_s, last_s);
                self.colors.swap(cur_e, last_e);
                self.durations.swap(i, len - 1);
                len -= 1;
            } else {
                i += 1;
            }
        }

        self.positions.truncate(len * 2);
        self.colors.truncate(len * 2);
        self.durations.truncate(len);
    }
}
