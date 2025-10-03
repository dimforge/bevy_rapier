use crate::math::Real;
use bevy::reflect::reflect_remote;
#[cfg(feature = "dim3")]
use rapier::dynamics::FrictionModel;
use rapier::dynamics::IntegrationParameters;

/// Friction models used for all contact constraints between two rigid-bodies.
///
/// This selection does not apply to multibodies that always rely on the [`FrictionModel::Coulomb`].
#[cfg(feature = "dim3")]
#[reflect_remote(FrictionModel)]
#[derive(Default, Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub enum FrictionModelWrapper {
    /// A simplified friction model significantly faster to solve than [`Self::Coulomb`]
    /// but less accurate.
    ///
    /// Instead of solving one Coulomb friction constraint per contact in a contact manifold,
    /// this approximation only solves one Coulomb friction constraint per group of 4 contacts
    /// in a contact manifold, plus one "twist" constraint. The "twist" constraint is purely
    /// rotational and aims to eliminate angular movement in the manifold’s tangent plane.
    #[default]
    Simplified,
    /// The coulomb friction model.
    ///
    /// This results in one Coulomb friction constraint per contact point.
    Coulomb,
}

#[cfg(not(feature = "dim3"))]
#[reflect_remote(IntegrationParameters)]
#[derive(Copy, Clone, Debug, PartialEq)]
/// Parameters for a time-step of the physics engine.
pub struct IntegrationParametersWrapper {
    /// The timestep length (default: `1.0 / 60.0`).
    pub dt: Real,
    /// Minimum timestep size when using CCD with multiple substeps (default: `1.0 / 60.0 / 100.0`).
    ///
    /// When CCD with multiple substeps is enabled, the timestep is subdivided
    /// into smaller pieces. This timestep subdivision won't generate timestep
    /// lengths smaller than `min_ccd_dt`.
    ///
    /// Setting this to a large value will reduce the opportunity to performing
    /// CCD substepping, resulting in potentially more time dropped by the
    /// motion-clamping mechanism. Setting this to an very small value may lead
    /// to numerical instabilities.
    pub min_ccd_dt: Real,

    /// > 0: the damping ratio used by the springs for contact constraint stabilization.
    ///
    /// Larger values make the constraints more compliant (allowing more visible
    /// penetrations before stabilization).
    /// (default `5.0`).
    pub contact_damping_ratio: Real,

    /// > 0: the natural frequency used by the springs for contact constraint regularization.
    ///
    /// Increasing this value will make it so that penetrations get fixed more quickly at the
    /// expense of potential jitter effects due to overshooting. In order to make the simulation
    /// look stiffer, it is recommended to increase the [`Self::contact_damping_ratio`] instead of this
    /// value.
    /// (default: `30.0`).
    pub contact_natural_frequency: Real,

    /// > 0: the natural frequency used by the springs for joint constraint regularization.
    ///
    /// Increasing this value will make it so that penetrations get fixed more quickly.
    /// (default: `1.0e6`).
    pub joint_natural_frequency: Real,

    /// The fraction of critical damping applied to the joint for constraints regularization.
    ///
    /// Larger values make the constraints more compliant (allowing more joint
    /// drift before stabilization).
    /// (default `1.0`).
    pub joint_damping_ratio: Real,

    /// The coefficient in `[0, 1]` applied to warmstart impulses, i.e., impulses that are used as the
    /// initial solution (instead of 0) at the next simulation step.
    ///
    /// This should generally be set to 1.
    ///
    /// (default `1.0`).
    pub warmstart_coefficient: Real,

    /// The approximate size of most dynamic objects in the scene.
    ///
    /// This value is used internally to estimate some length-based tolerance. In particular, the
    /// values [`IntegrationParameters::allowed_linear_error`],
    /// [`IntegrationParameters::max_corrective_velocity`],
    /// [`IntegrationParameters::prediction_distance`], [`RigidBodyActivation::normalized_linear_threshold`]
    /// are scaled by this value implicitly.
    ///
    /// This value can be understood as the number of units-per-meter in your physical world compared
    /// to a human-sized world in meter. For example, in a 2d game, if your typical object size is 100
    /// pixels, set the [`Self::length_unit`] parameter to 100.0. The physics engine will interpret
    /// it as if 100 pixels is equivalent to 1 meter in its various internal threshold.
    /// (default `1.0`).
    pub length_unit: Real,

    /// Amount of penetration the engine won’t attempt to correct (default: `0.001m`).
    ///
    /// This value is implicitly scaled by [`IntegrationParameters::length_unit`].
    pub normalized_allowed_linear_error: Real,
    /// Maximum amount of penetration the solver will attempt to resolve in one timestep (default: `10.0`).
    ///
    /// This value is implicitly scaled by [`IntegrationParameters::length_unit`].
    pub normalized_max_corrective_velocity: Real,
    /// The maximal distance separating two objects that will generate predictive contacts (default: `0.002m`).
    ///
    /// This value is implicitly scaled by [`IntegrationParameters::length_unit`].
    pub normalized_prediction_distance: Real,
    /// The number of solver iterations run by the constraints solver for calculating forces (default: `4`).
    pub num_solver_iterations: usize,
    /// Number of internal Project Gauss Seidel (PGS) iterations run at each solver iteration (default: `1`).
    pub num_internal_pgs_iterations: usize,
    /// The number of stabilization iterations run at each solver iterations (default: `2`).
    pub num_internal_stabilization_iterations: usize,
    /// Minimum number of dynamic bodies in each active island (default: `128`).
    pub min_island_size: usize,
    /// Maximum number of substeps performed by the  solver (default: `1`).
    pub max_ccd_substeps: usize,
}

#[cfg(feature = "dim3")]
#[reflect_remote(IntegrationParameters)]
#[derive(Copy, Clone, Debug, PartialEq)]
/// Parameters for a time-step of the physics engine.
pub struct IntegrationParametersWrapper {
    /// The timestep length (default: `1.0 / 60.0`).
    pub dt: Real,
    /// Minimum timestep size when using CCD with multiple substeps (default: `1.0 / 60.0 / 100.0`).
    ///
    /// When CCD with multiple substeps is enabled, the timestep is subdivided
    /// into smaller pieces. This timestep subdivision won't generate timestep
    /// lengths smaller than `min_ccd_dt`.
    ///
    /// Setting this to a large value will reduce the opportunity to performing
    /// CCD substepping, resulting in potentially more time dropped by the
    /// motion-clamping mechanism. Setting this to an very small value may lead
    /// to numerical instabilities.
    pub min_ccd_dt: Real,

    /// > 0: the damping ratio used by the springs for contact constraint stabilization.
    ///
    /// Larger values make the constraints more compliant (allowing more visible
    /// penetrations before stabilization).
    /// (default `5.0`).
    pub contact_damping_ratio: Real,

    /// > 0: the natural frequency used by the springs for contact constraint regularization.
    ///
    /// Increasing this value will make it so that penetrations get fixed more quickly at the
    /// expense of potential jitter effects due to overshooting. In order to make the simulation
    /// look stiffer, it is recommended to increase the [`Self::contact_damping_ratio`] instead of this
    /// value.
    /// (default: `30.0`).
    pub contact_natural_frequency: Real,

    /// > 0: the natural frequency used by the springs for joint constraint regularization.
    ///
    /// Increasing this value will make it so that penetrations get fixed more quickly.
    /// (default: `1.0e6`).
    pub joint_natural_frequency: Real,

    /// The fraction of critical damping applied to the joint for constraints regularization.
    ///
    /// Larger values make the constraints more compliant (allowing more joint
    /// drift before stabilization).
    /// (default `1.0`).
    pub joint_damping_ratio: Real,

    /// The coefficient in `[0, 1]` applied to warmstart impulses, i.e., impulses that are used as the
    /// initial solution (instead of 0) at the next simulation step.
    ///
    /// This should generally be set to 1.
    ///
    /// (default `1.0`).
    pub warmstart_coefficient: Real,

    /// The approximate size of most dynamic objects in the scene.
    ///
    /// This value is used internally to estimate some length-based tolerance. In particular, the
    /// values [`IntegrationParameters::allowed_linear_error`],
    /// [`IntegrationParameters::max_corrective_velocity`],
    /// [`IntegrationParameters::prediction_distance`], [`RigidBodyActivation::normalized_linear_threshold`]
    /// are scaled by this value implicitly.
    ///
    /// This value can be understood as the number of units-per-meter in your physical world compared
    /// to a human-sized world in meter. For example, in a 2d game, if your typical object size is 100
    /// pixels, set the [`Self::length_unit`] parameter to 100.0. The physics engine will interpret
    /// it as if 100 pixels is equivalent to 1 meter in its various internal threshold.
    /// (default `1.0`).
    pub length_unit: Real,

    /// Amount of penetration the engine won’t attempt to correct (default: `0.001m`).
    ///
    /// This value is implicitly scaled by [`IntegrationParameters::length_unit`].
    pub normalized_allowed_linear_error: Real,
    /// Maximum amount of penetration the solver will attempt to resolve in one timestep (default: `10.0`).
    ///
    /// This value is implicitly scaled by [`IntegrationParameters::length_unit`].
    pub normalized_max_corrective_velocity: Real,
    /// The maximal distance separating two objects that will generate predictive contacts (default: `0.002m`).
    ///
    /// This value is implicitly scaled by [`IntegrationParameters::length_unit`].
    pub normalized_prediction_distance: Real,
    /// The number of solver iterations run by the constraints solver for calculating forces (default: `4`).
    pub num_solver_iterations: usize,
    /// Number of internal Project Gauss Seidel (PGS) iterations run at each solver iteration (default: `1`).
    pub num_internal_pgs_iterations: usize,
    /// The number of stabilization iterations run at each solver iterations (default: `2`).
    pub num_internal_stabilization_iterations: usize,
    /// Minimum number of dynamic bodies in each active island (default: `128`).
    pub min_island_size: usize,
    /// Maximum number of substeps performed by the  solver (default: `1`).
    pub max_ccd_substeps: usize,
    /// The friction model used by the solver (default: `FrictionModel::Rigid`).
    #[reflect(remote = FrictionModelWrapper)]
    pub friction_model: FrictionModel,
}
