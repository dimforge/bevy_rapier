use crate::physics;
use crate::physics::{
    JointsEntityMap, ModificationTracker, PhysicsHooksWithQueryObject, RapierConfiguration,
    SimulationToRenderTime,
};
use crate::prelude::IntersectionEvent;
use crate::rapier::geometry::ContactEvent;
use crate::rapier::pipeline::QueryPipeline;
use bevy::app::Events;
use bevy::ecs::query::WorldQuery;
use bevy::prelude::*;
use rapier::dynamics::{CCDSolver, IntegrationParameters, IslandManager, JointSet};
use rapier::geometry::{BroadPhase, NarrowPhase};
use rapier::pipeline::PhysicsPipeline;
use std::marker::PhantomData;

pub type NoUserData<'a> = &'a ();

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
pub struct RapierPhysicsPlugin<UserData>(PhantomData<UserData>);

impl<UserData> Default for RapierPhysicsPlugin<UserData> {
    fn default() -> Self {
        Self(PhantomData)
    }
}

/// The stage where the physics transform are output to the Bevy Transform.
///
/// This stage is added right before the `POST_UPDATE` stage.
pub const TRANSFORM_SYNC_STAGE: &'static str = "rapier::transform_sync_stage";

/// The names of the default App stages
#[derive(Debug, Hash, PartialEq, Eq, Clone, StageLabel)]
pub enum PhysicsStages {
    FinalizeCreations,
    SyncTransforms,
}

impl<UserData: 'static + WorldQuery + Send + Sync> Plugin for RapierPhysicsPlugin<UserData> {
    fn build(&self, app: &mut AppBuilder) {
        app.add_stage_before(
            CoreStage::PreUpdate,
            PhysicsStages::FinalizeCreations,
            SystemStage::parallel(),
        )
        .add_stage_before(
            CoreStage::PostUpdate,
            PhysicsStages::SyncTransforms,
            SystemStage::parallel(),
        )
        .insert_resource(PhysicsPipeline::new())
        .insert_resource(QueryPipeline::new())
        .insert_resource(RapierConfiguration::default())
        .insert_resource(IntegrationParameters::default())
        .insert_resource(BroadPhase::new())
        .insert_resource(NarrowPhase::new())
        .insert_resource(IslandManager::new())
        .insert_resource(JointSet::new())
        .insert_resource(CCDSolver::new())
        .insert_resource(PhysicsHooksWithQueryObject::<UserData>(Box::new(())))
        .insert_resource(Events::<IntersectionEvent>::default())
        .insert_resource(Events::<ContactEvent>::default())
        .insert_resource(SimulationToRenderTime::default())
        .insert_resource(JointsEntityMap::default())
        .insert_resource(ModificationTracker::default())
        .add_system_to_stage(
            PhysicsStages::FinalizeCreations,
            physics::attach_bodies_and_colliders_system
                .system()
                .label(physics::ATTACH_BODIES_AND_COLLIDERS_SYSTEM),
        )
        .add_system_to_stage(
            PhysicsStages::FinalizeCreations,
            physics::create_joints_system
                .system()
                .label(physics::CREATE_JOINTS_SYSTEM),
        )
        .add_system_to_stage(
            CoreStage::PreUpdate,
            physics::finalize_collider_attach_to_bodies
                .system()
                .label(physics::FINALIZE_COLLIDER_ATTACH_TO_BODIES_SYSTEM),
        )
        .add_system_to_stage(
            CoreStage::Update,
            physics::step_world_system::<UserData>
                .system()
                .label(physics::STEP_WORLD_SYSTEM),
        )
        .add_system_to_stage(
            PhysicsStages::SyncTransforms,
            physics::sync_transforms
                .system()
                .label(physics::SYNC_TRANSFORMS_SYSTEM),
        )
        .add_system_to_stage(
            CoreStage::PostUpdate,
            physics::collect_removals
                .system()
                .label(physics::COLLECT_REMOVALS_SYSTEM),
        );
    }
}
