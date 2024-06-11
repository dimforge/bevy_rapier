use crate::prelude::PhysicsWorld;

pub use self::configuration::{RapierConfiguration, SimulationToRenderTime, TimestepMode};
pub use self::context::RapierContext;
pub use self::plugin::{
    NoUserData, PhysicsSet, RapierPhysicsPlugin, RapierTransformPropagateSet, RapierWorld, WorldId,
    DEFAULT_WORLD_ID,
};

#[allow(clippy::type_complexity)]
#[allow(clippy::too_many_arguments)]
pub mod systems;

mod configuration;
pub(crate) mod context;
mod narrow_phase;
#[allow(clippy::module_inception)]
pub(crate) mod plugin;

fn get_world<'a>(
    world_within: Option<&'a PhysicsWorld>,
    context: &'a mut RapierContext,
) -> &'a mut RapierWorld {
    let world_id = world_within.map(|x| x.world_id).unwrap_or(DEFAULT_WORLD_ID);

    context
        .get_world_mut(world_id)
        .expect("World {world_id} does not exist")
}

fn find_item_and_world<T>(
    context: &mut RapierContext,
    item_finder: impl Fn(&mut RapierWorld) -> Option<T>,
) -> Option<(&mut RapierWorld, T)> {
    for (_, world) in context.worlds.iter_mut() {
        if let Some(handle) = item_finder(world) {
            return Some((world, handle));
        }
    }

    None
}
