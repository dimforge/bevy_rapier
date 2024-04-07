use bevy::prelude::Resource;

use crate::math::Vect;

#[derive(Resource, Copy, Clone, Debug)]
/// A resource for specifying configuration information for the physics simulation
pub struct RapierConfiguration {
    /// Specifying the gravity of the physics simulation.
    pub gravity: Vect,
    /// Specifies if the physics simulation is active and update the physics world.
    pub physics_pipeline_active: bool,
    /// Specifies if the query pipeline is active and update the query pipeline.
    pub query_pipeline_active: bool,
    /// Specifies the number of subdivisions along each axes a shape should be subdivided
    /// if its scaled representation cannot be represented with the same shape type.
    ///
    /// For example, a ball subject to a non-uniform scaling cannot be represented as a ball
    /// (it’s an ellipsoid). Thus, in order to be compatible with Rapier, the shape is automatically
    /// discretized into a convex polyhedron, using `scaled_shape_subdivision` as the number of subdivisions
    /// along each spherical coordinates angle.
    pub scaled_shape_subdivision: u32,
    /// Specifies if backend sync should always accept tranform changes, which may be from the writeback stage.
    pub force_update_from_transform_changes: bool,
    /// The number of substeps that will be performed at each tick.
    pub substeps: usize,
}

impl Default for RapierConfiguration {
    fn default() -> Self {
        Self {
            #[cfg(feature = "dim2")]
            gravity: Vect::Y * -9.81 * 10.0,
            #[cfg(feature = "dim3")]
            gravity: Vect::Y * -9.81,
            physics_pipeline_active: true,
            query_pipeline_active: true,
            scaled_shape_subdivision: 10,
            force_update_from_transform_changes: false,
            substeps: 1,
        }
    }
}
