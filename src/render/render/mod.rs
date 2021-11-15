mod wireframe_material;
pub use self::wireframe_material::WireframeMaterial;

mod position_material;
pub use self::position_material::PositionWireframeMaterial;

mod setup;
pub use self::setup::setup_material;

mod pass;
pub use self::pass::setup_debug_pass;

use bevy::prelude::*;
use bevy::reflect::TypeUuid;
use bevy::render::pipeline::PipelineDescriptor;

/// Rapier Debug Render Pass.
/// **Note:* Runs after the Main pass before the Ui Pass.
#[derive(Debug, Clone, Default, bevy::render::renderer::RenderResources)]
pub struct RapierDebugPass;

/// Name of the camera used in th rapier_debug render pass.
pub const CAMERA_RAPIER_DEBUG: &str = "camera_rapier_debug";
/// Name of the rapier_debug render pass.
pub const RAPIER_DEBUG_PASS: &str = "rapier_debug_pass";
/// Mesh identifier used in the rapier_debug render pass.
pub const RAPIER_DEBUG_MESH: &str = "rapier_debug_mesh";

/// Static handle to the Collider render pipeline.
pub const COLLIDER_PIPELINE_HANDLE: HandleUntyped =
    HandleUntyped::weak_from_u64(PipelineDescriptor::TYPE_UUID, 0x936896ad9d35720c_u64);

/// Static handle to the Position render pipeline.
pub const POSITION_PIPELINE_HANDLE: HandleUntyped =
    HandleUntyped::weak_from_u64(PipelineDescriptor::TYPE_UUID, 0x0807a09e1c05e8b0_u64);

/// Handle to the Collider pipieline. Used to access the collider pipeline.
pub struct ColliderWireframePipelineDescriptor(bevy::asset::Handle<bevy::render::pipeline::PipelineDescriptor>);
/// Handle to the positions pipeline. Useful for accessing the position pipeline.
pub struct PositionPipelineDescriptor(bevy::asset::Handle<bevy::render::pipeline::PipelineDescriptor>);
