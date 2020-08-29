use crate::physics;
use crate::physics::{EventQueue, Gravity, RapierPhysicsScale};
use bevy::prelude::*;
use rapier::dynamics::{IntegrationParameters, JointSet, RigidBodySet};
use rapier::geometry::{BroadPhase, ColliderSet, NarrowPhase};
use rapier::math::Vector;
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

const BUILD_RIGID_BODIES: &'static str = "build_rigid_bodies_stage";

impl Plugin for RapierPhysicsPlugin {
    fn build(&self, app: &mut AppBuilder) {
        app.add_resource(PhysicsPipeline::new())
            .add_resource(IntegrationParameters::default())
            .add_resource(Gravity(Vector::y() * -9.81))
            .add_resource(BroadPhase::new())
            .add_resource(NarrowPhase::new())
            .add_resource(RigidBodySet::new())
            .add_resource(ColliderSet::new())
            .add_resource(JointSet::new())
            .add_resource(RapierPhysicsScale(1.0))
            .add_resource(EventQueue::new(true))
            .add_stage_before(stage::PRE_UPDATE, BUILD_RIGID_BODIES)
            // .add_system_to_stage(BUILD_RIGID_BODIES, physics::create_bodies_system.system())
            .add_system_to_stage(BUILD_RIGID_BODIES, physics::add_bodies_system.system())
            // .add_system_to_stage(stage::PRE_UPDATE, physics::create_colliders_system.system())
            .add_system_to_stage(stage::PRE_UPDATE, physics::create_joints_system.system())
            .add_system_to_stage(stage::PRE_UPDATE, physics::add_colliders_system.system())
            .add_system_to_stage(stage::UPDATE, physics::step_world_system.system())
            .add_system_to_stage(stage::POST_UPDATE, physics::sync_transform_system.system());
    }
}
