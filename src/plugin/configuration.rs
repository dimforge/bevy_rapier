use bevy::prelude::{FromWorld, Resource, World};

use crate::math::{Real, Vect};
use crate::plugin::RapierContext;

use super::RapierWorld;

/// Difference between simulation and rendering time
#[derive(Resource, Default)]
pub struct SimulationToRenderTime {
    /// Difference between simulation and rendering time
    pub diff: f32,
}

/// The different ways of adjusting the timestep length.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum TimestepMode {
    /// Use a fixed timestep: the physics simulation will be advanced by the fixed value
    /// `dt` seconds at each Bevy tick by performing `substeps` of length `dt / substeps`.
    Fixed {
        /// The physics simulation will be advanced by this total amount at each Bevy tick.
        dt: f32,
        /// This number of substeps of length `dt / substeps` will be performed at each Bevy tick.
        substeps: usize,
    },
    /// Use a variable timestep: the physics simulation will be advanced by the variable value
    /// `min(max_dt, Time::delta_seconds() * time_scale)` seconds at each Bevy tick. If
    /// `time_scale > 1.0` then the simulation will appear to run faster than real-time whereas
    /// `time_scale < 1.0` makes the simulation run in slow-motion.
    Variable {
        /// Maximum amount of time the physics simulation may be advanced at each Bevy tick.
        max_dt: f32,
        /// Multiplier controlling if the physics simulation should advance faster (> 1.0),
        /// at the same speed (= 1.0) or slower (< 1.0) than the real time.
        time_scale: f32,
        /// The number of substeps that will be performed at each tick.
        substeps: usize,
    },
    /// Use a fixed timestep equal to `IntegrationParameters::dt`, but don't step if the
    /// physics simulation advanced by a time greater than the real-world elapsed time multiplied by `time_scale`.
    /// Rigid-bodies with a component `InterpolatedTransform` attached will use interpolation to
    /// estimate the rigid-bodies position in-between steps.
    Interpolated {
        /// The physics simulation will be advanced by this total amount at each Bevy tick, unless
        /// the physics simulation time is ahead of a the real time.
        dt: f32,
        /// Multiplier controlling if the physics simulation should advance faster (> 1.0),
        /// at the same speed (= 1.0) or slower (< 1.0) than the real time.
        time_scale: f32,
        /// The number of substeps that will be performed whenever the physics simulation is advanced.
        substeps: usize,
    },
}

#[derive(Resource, Copy, Clone, Debug)]
/// A resource for specifying configuration information for the physics simulation
pub struct RapierConfiguration {
    /// Specifying the gravity of the physics simulation.
    pub gravity: Vect,
    /// Specifies if the physics simulation is active and update the physics world.
    pub physics_pipeline_active: bool,
    /// Specifies if the query pipeline is active and update the query pipeline.
    pub query_pipeline_active: bool,
    /// Specifies the way the timestep length should be adjusted at each frame.
    pub timestep_mode: TimestepMode,
    /// Specifies the number of subdivisions along each axes a shape should be subdivided
    /// if its scaled representation cannot be represented with the same shape type.
    ///
    /// For example, a ball subject to a non-uniform scaling cannot be represented as a ball
    /// (it’s an ellipsoid). Thus, in order to be compatible with Rapier, the shape is automatically
    /// discretized into a convex polyhedron, using `scaled_shape_subdivision` as the number of subdivisions
    /// along each spherical coordinates angle.
    pub scaled_shape_subdivision: u32,
    /// Specifies if backend sync should always accept transform changes, which may be from the writeback stage.
    pub force_update_from_transform_changes: bool,
}

impl RapierWorld {
    pub fn rapier_configuration(&self) -> RapierConfiguration {
        RapierConfiguration::new(self.integration_parameters.length_unit)
    }
}

impl RapierConfiguration {
    /// Configures rapier with the specified length unit.
    ///
    /// See the documentation of [`IntegrationParameters::length_unit`] for additional details
    /// on that argument.
    ///
    /// The default gravity is automatically scaled by that length unit.
    pub fn new(length_unit: Real) -> Self {
        Self {
            gravity: Vect::Y * -9.81 * length_unit,
            physics_pipeline_active: true,
            query_pipeline_active: true,
            timestep_mode: TimestepMode::Variable {
                max_dt: 1.0 / 60.0,
                time_scale: 1.0,
                substeps: 1,
            },
            scaled_shape_subdivision: 10,
            force_update_from_transform_changes: false,
        }
    }
}
