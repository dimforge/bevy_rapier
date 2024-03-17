use crate::dynamics::{RigidBodyHandle, TransformInterpolation};
use crate::pipeline::{CollisionEvent, ContactForceEvent};
use crate::plugin::configuration::SimulationToRenderTime;
use crate::plugin::{RapierConfiguration, RapierContext};
use crate::prelude::{BevyPhysicsHooks, BevyPhysicsHooksAdapter};
use bevy::ecs::system::{StaticSystemParam, SystemParamItem};
use bevy::prelude::*;

#[cfg(feature = "dim2")]
use bevy::math::Vec3Swizzles;

/// System responsible for advancing the physics simulation, and updating the internal state
/// for scene queries.
pub fn step_simulation<Hooks>(
    mut context: ResMut<RapierContext>,
    config: Res<RapierConfiguration>,
    hooks: StaticSystemParam<Hooks>,
    time: Res<Time>,
    mut sim_to_render_time: ResMut<SimulationToRenderTime>,
    collision_events: EventWriter<CollisionEvent>,
    contact_force_events: EventWriter<ContactForceEvent>,
    interpolation_query: Query<(Entity, &mut TransformInterpolation)>,
) where
    Hooks: 'static + BevyPhysicsHooks,
    for<'w, 's> SystemParamItem<'w, 's, Hooks>: BevyPhysicsHooks,
{
    let context = &mut *context;
    let hooks_adapter = BevyPhysicsHooksAdapter::new(hooks.into_inner());

    if config.physics_pipeline_active {
        context.step_simulation(
            config.gravity,
            config.timestep_mode,
            Some((collision_events, contact_force_events)),
            &hooks_adapter,
            &time,
            &mut sim_to_render_time,
            Some(interpolation_query),
        );
        context.deleted_colliders.clear();
    } else {
        context.propagate_modified_body_positions_to_colliders();
    }

    if config.query_pipeline_active {
        context.update_query_pipeline();
    }
}
