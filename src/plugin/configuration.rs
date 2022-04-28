use crate::math::Vect;

/// Difference between simulation and rendering time
#[derive(Default)]
pub struct SimulationToRenderTime {
    /// Difference between simulation and rendering time
    pub diff: f32,
}

/// The different ways of adjusting the timestep length.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum TimestepMode {
    /// Use a fixed timestep: the physics simulation will be advanced by the fixed value
    /// `dt` seconds at each Bevy tick by performing `substeps` of length `dt / substeps`.
    Fixed { dt: f32, substeps: usize },
    /// Use a variable timestep: the physics simulation will be advanced by the variable value
    /// `min(max_dt, Time::delta_seconds() * time_scale)` seconds at each Bevy tick. If
    /// `time_scale > 1.0` then the simulation will appear to run faster than real-time whereas
    /// `time_scale < 1.0` makes the simulation run in slow-motion.
    Variable {
        max_dt: f32,
        time_scale: f32,
        substeps: usize,
    },
    /// Use a fixed timestep equal to `IntegrationParameters::dt`, but don't step if the
    /// physics simulation advanced by a time greater than the real-world elapsed time multiplied by `time_scale`.
    /// Rigid-bodies with a component `InterpolatedTransform` attached will use interpolation to
    /// estimate the rigid-bodies position in-between steps.
    Interpolated {
        dt: f32,
        time_scale: f32,
        substeps: usize,
    },
}

#[derive(Copy, Clone, Debug)]
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
            timestep_mode: TimestepMode::Variable {
                max_dt: 1.0 / 60.0,
                time_scale: 1.0,
                substeps: 1,
            },
            scaled_shape_subdivision: 10,
        }
    }
}
