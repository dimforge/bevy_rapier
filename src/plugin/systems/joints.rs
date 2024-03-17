use crate::dynamics::{
    ImpulseJoint, ImpulseJointCreated, ImpulseJointHandle, MultibodyJoint, MultibodyJointCreated,
    MultibodyJointHandle, RigidBody, RigidBodyCreated,
};
use crate::plugin::RapierContext;
use bevy::prelude::*;

#[cfg(feature = "dim2")]
use bevy::math::Vec3Swizzles;

/// System responsible for creating new Rapier joints from the related `bevy_rapier` components.
pub fn init_joints(
    mut commands: Commands,
    mut context: ResMut<RapierContext>,
    impulse_joints: Query<(Entity, &ImpulseJoint), Without<ImpulseJointCreated>>,
    multibody_joints: Query<(Entity, &MultibodyJoint), Without<MultibodyJointCreated>>,
    parent_query: Query<&Parent>,
    body_exists: Query<&RigidBodyCreated>,
) {
    let context = &mut *context;

    for (entity, joint) in impulse_joints.iter() {
        let mut target = None;
        let mut body_entity = entity;
        while target.is_none() {
            target = body_exists.get(body_entity).ok().map(|_| body_entity);
            if let Ok(parent_entity) = parent_query.get(body_entity) {
                body_entity = parent_entity.get();
            } else {
                break;
            }
        }

        if let (Some(target), Some(source)) = (
            target,
            body_exists.get(joint.parent).ok().map(|_| joint.parent),
        ) {
            let handle = context
                .impulse_joints
                .insert(entity, source, target, joint.data, true);
            commands.entity(entity).insert(ImpulseJointCreated);
        }
    }

    for (entity, joint) in multibody_joints.iter() {
        let target = body_exists.get(entity).ok().map(|_| entity);
        let source = body_exists.get(joint.parent).ok().map(|_| joint.parent);

        if let (Some(target), Some(source)) = (target, source) {
            if let Some(handle) = context
                .multibody_joints
                .insert(source, target, joint.data, true)
            {
                commands.entity(entity).insert(MultibodyJointCreated);
            } else {
                error!("Failed to create multibody joint: loop detected.")
            }
        }
    }
}

/// System responsible for applying changes the user made to a joint component.
pub fn apply_joint_user_changes(
    mut context: ResMut<RapierContext>,
    changed_impulse_joints: Query<(Entity, &ImpulseJoint), Changed<ImpulseJoint>>,
    changed_multibody_joints: Query<(Entity, &MultibodyJoint), Changed<MultibodyJoint>>,
) {
    // TODO: right now, we only support propagating changes made to the joint data.
    //       Re-parenting the joint isn’t supported yet.
    for (handle, changed_joint) in changed_impulse_joints.iter() {
        if let Some(joint) = context.impulse_joints.get_mut(handle) {
            joint.data = changed_joint.data;
        }
    }

    for (handle, changed_joint) in changed_multibody_joints.iter() {
        // TODO: not sure this will always work properly, e.g., if the number of Dofs is changed.
        if let Some((mb, link_id)) = context.multibody_joints.get_mut(handle) {
            if let Some(link) = mb.link_mut(link_id) {
                link.joint.data = changed_joint.data;
            }
        }
    }
}
