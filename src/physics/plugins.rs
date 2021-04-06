use crate::physics;
use crate::physics::{
    EntityMaps, EventQueue, InteractionPairFilters, RapierConfiguration, SimulationToRenderTime,
};
use crate::rapier::pipeline::QueryPipeline;
use bevy::prelude::*;
use rapier::dynamics::{CCDSolver, IntegrationParameters, JointSet, RigidBodySet};
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

/// The stage where the physics transform are output to the Bevy Transform.
///
/// This stage is added right before the `POST_UPDATE` stage.
pub const TRANSFORM_SYNC_STAGE: &'static str = "rapier::transform_sync_stage";

impl Plugin for RapierPhysicsPlugin {
    fn build(&self, app: &mut AppBuilder) {
        app.insert_resource(PhysicsPipeline::new())
            .insert_resource(QueryPipeline::new())
            .insert_resource(RapierConfiguration::default())
            .insert_resource(IntegrationParameters::default())
            .insert_resource(BroadPhase::new())
            .insert_resource(NarrowPhase::new())
            .insert_resource(RigidBodySet::new())
            .insert_resource(ColliderSet::new())
            .insert_resource(JointSet::new())
            .insert_resource(CCDSolver::new())
            .insert_resource(InteractionPairFilters::new())
            .insert_resource(EventQueue::new(true))
            .insert_resource(SimulationToRenderTime::default())
            .insert_resource(EntityMaps::default())
            // TODO: can we avoid this map? We are only using this
            // to avoid some borrowing issue when joints creations
            // are needed.
            .add_system_to_stage(
                CoreStage::PreUpdate,
                physics::create_body_and_collider_system.system(),
            )
            .add_system_to_stage(
                CoreStage::PreUpdate,
                physics::update_collider_system.system(),
            )
            .add_system_to_stage(CoreStage::PreUpdate, physics::create_joints_system.system())
            .add_system_to_stage(CoreStage::Update, physics::step_world_system.system())
            .add_stage_before(
                CoreStage::PostUpdate,
                TRANSFORM_SYNC_STAGE,
                SystemStage::parallel(),
            )
            .add_system_to_stage(
                TRANSFORM_SYNC_STAGE,
                physics::sync_transform_system.system(),
            )
            .add_system_to_stage(
                TRANSFORM_SYNC_STAGE,
                physics::destroy_body_and_collider_system.system(),
            );
    }
}
