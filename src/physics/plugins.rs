use crate::physics;
use crate::physics::{EntityToBody, EventQueue, Gravity, RapierPhysicsScale};
use bevy::prelude::*;
use rapier::dynamics::{IntegrationParameters, JointSet, RigidBodySet};
use rapier::geometry::{BroadPhase, ColliderSet, NarrowPhase};
use rapier::math::Vector;
use rapier::pipeline::PhysicsPipeline;

pub struct RapierPhysicsPlugin;

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
            // TODO: can we avoid this map? We are only using this
            // to avoid some borrowing issue when joints creations
            // are needed.
            .add_resource(EntityToBody::new())
            .add_system_to_stage_front(
                stage::PRE_UPDATE,
                physics::create_body_and_collider_system.system(),
            )
            .add_system_to_stage(stage::PRE_UPDATE, physics::create_joints_system.system())
            .add_system_to_stage(stage::UPDATE, physics::step_world_system.system())
            .add_system_to_stage(stage::POST_UPDATE, physics::sync_transform_system.system());
    }
}
