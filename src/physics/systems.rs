use crate::physics::{
    ColliderHandleComponent, EventQueue, Gravity, JointBuilderComponent, JointHandleComponent,
    RapierPhysicsScale, RigidBodyHandleComponent,
};

use crate::rapier::dynamics::RigidBody;
use crate::rapier::geometry::Collider;
use bevy::ecs::Mut;
use bevy::prelude::*;
use rapier::dynamics::{IntegrationParameters, JointSet, RigidBodySet};
use rapier::geometry::{BroadPhase, ColliderSet, NarrowPhase};
use rapier::pipeline::PhysicsPipeline;

/// System responsible for creating a rigid-body on the `RigidBodySet` using the `RigidBody` component.
pub fn add_bodies_system(
    mut commands: Commands,
    mut bodies: ResMut<RigidBodySet>,
    entity: Entity,
    body: Added<RigidBody>,
) {
    let handle = bodies.insert((*body).clone());
    commands.insert_one(entity, RigidBodyHandleComponent::from(handle));
}

/// System responsible for creating a collider on the `ColliderSet` using the `Collider` component.
pub fn add_colliders_system(
    mut commands: Commands,
    mut colliders: ResMut<ColliderSet>,
    mut bodies: ResMut<RigidBodySet>,
    entity: Entity,
    parent: &RigidBodyHandleComponent,
    collider: Added<Collider>,
) {
    let handle = colliders.insert(collider.clone(), parent.handle(), &mut *bodies);
    commands.insert_one(entity, ColliderHandleComponent::from(handle));
}

/// System responsible for creating Rapier joints from their builder resources.
pub fn create_joints_system(
    mut commands: Commands,
    mut bodies: ResMut<RigidBodySet>,
    mut joints: ResMut<JointSet>,
    mut joint_builders: Query<(Entity, &JointBuilderComponent)>,
    handles: Query<&RigidBodyHandleComponent>,
) {
    for (entity, joint) in &mut joint_builders.iter() {
        let body1 = handles
            .get::<RigidBodyHandleComponent>(joint.entity1)
            .unwrap();
        let body2 = handles
            .get::<RigidBodyHandleComponent>(joint.entity2)
            .unwrap();

        // if let (Ok(body1), Ok(body2)) = (body1, body2) {
        let handle = joints.insert(&mut bodies, body1.handle(), body2.handle(), joint.params);
        commands.insert_one(
            entity,
            JointHandleComponent::new(handle, joint.entity1, joint.entity2),
        );
        commands.remove_one::<JointBuilderComponent>(entity);
        // }
    }
}

/// System responsible for performing one timestep of the physics world.
pub fn step_world_system(
    gravity: Res<Gravity>,
    integration_parameters: Res<IntegrationParameters>,
    mut pipeline: ResMut<PhysicsPipeline>,
    mut broad_phase: ResMut<BroadPhase>,
    mut narrow_phase: ResMut<NarrowPhase>,
    mut bodies: ResMut<RigidBodySet>,
    mut colliders: ResMut<ColliderSet>,
    mut joints: ResMut<JointSet>,
    events: Res<EventQueue>,
) {
    if events.auto_clear {
        events.clear();
    }

    pipeline.step(
        &gravity.0,
        &integration_parameters,
        &mut broad_phase,
        &mut narrow_phase,
        &mut bodies,
        &mut colliders,
        &mut joints,
        &*events,
    );
}

/// System responsible for writing the rigid-bodies positions into the Bevy translation and rotation components.
pub fn sync_transform_system(
    bodies: ResMut<RigidBodySet>,
    scale: Res<RapierPhysicsScale>,
    rigid_body: &RigidBodyHandleComponent,
    mut translation: Mut<Translation>,
    mut rotation: Mut<Rotation>,
) {
    if let Some(rb) = bodies.get(rigid_body.handle()) {
        let pos = rb.position;

        #[cfg(feature = "dim2")]
        {
            let rot = na::UnitQuaternion::new(na::Vector3::z() * pos.rotation.angle());

            translation.0 =
                Vec3::new(pos.translation.vector.x, pos.translation.vector.y, 0.0) * scale.0;
            rotation.0 = Quat::from_xyzw(rot.i, rot.j, rot.k, rot.w);
        }

        #[cfg(feature = "dim3")]
        {
            translation.0 = Vec3::new(
                pos.translation.vector.x,
                pos.translation.vector.y,
                pos.translation.vector.z,
            ) * scale.0;

            rotation.0 = Quat::from_xyzw(
                pos.rotation.i,
                pos.rotation.j,
                pos.rotation.k,
                pos.rotation.w,
            );
        }
    }
}
