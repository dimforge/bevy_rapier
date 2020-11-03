use crate::physics;
use crate::physics::{
    EventQueue, InteractionPairFilters, RapierConfiguration, SimulationToRenderTime,
};
use crate::rapier::pipeline::QueryPipeline;
use bevy::prelude::*;
use rapier::dynamics::{IntegrationParameters, JointSet, RigidBodySet};
use rapier::geometry::{BroadPhase, ColliderSet, NarrowPhase};
use rapier::pipeline::PhysicsPipeline;

/// A plugin responsible for setting up a full Rapier physics simulation pipeline and resources.
///
/// This will automatically setup all the resources needed to run a Rapier physics simulation including:
/// - The physics pipeline.
/// - The integration parameters.
/// - The rigid-body, collider, and joint, sets.
/// - The gravity.
/// - The broad phase and narrow-phase.
/// - The event queue.
/// - Systems responsible for executing one physics timestep at each Bevy update stage.
pub struct RapierPhysicsPlugin;

impl Plugin for RapierPhysicsPlugin {
    fn build(&self, app: &mut AppBuilder) {
        app.add_resource(PhysicsPipeline::new())
            .add_resource(QueryPipeline::new())
            .add_resource(RapierConfiguration::default())
            .add_resource(IntegrationParameters::default())
            .add_resource(BroadPhase::new())
            .add_resource(NarrowPhase::new())
            .add_resource(RigidBodySet::new())
            .add_resource(ColliderSet::new())
            .add_resource(JointSet::new())
            .add_resource(InteractionPairFilters::new())
            .add_resource(EventQueue::new(true))
            .add_resource(SimulationToRenderTime::default())
            // TODO: can we avoid this map? We are only using this
            // to avoid some borrowing issue when joints creations
            // are needed.
            .add_system_to_stage_front(
                stage::PRE_UPDATE,
                physics::create_body_and_collider_system.system(),
            )
            .add_system_to_stage(stage::PRE_UPDATE, physics::create_joints_system.system())
            .add_system_to_stage(stage::UPDATE, physics::step_world_system.system())
            .add_system_to_stage(stage::POST_UPDATE, physics::sync_transform_system.system());
    }
}
