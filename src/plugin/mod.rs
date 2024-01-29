pub use self::configuration::RapierConfiguration;
pub use self::context::RapierContext;
pub use self::plugin::{NoUserData, PhysicsSet, RapierPhysicsPlugin, RapierTransformPropagateSet};

#[allow(clippy::type_complexity)]
#[allow(clippy::too_many_arguments)]
pub mod systems;

mod configuration;
mod context;
mod interpolation_context;
mod narrow_phase;
#[allow(clippy::module_inception)]
mod plugin;
