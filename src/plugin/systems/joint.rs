use crate::dynamics::ImpulseJoint;
use crate::dynamics::MultibodyJoint;
use crate::dynamics::RapierImpulseJointHandle;
use crate::dynamics::RapierMultibodyJointHandle;
use crate::plugin::RapierContext;
use bevy::prelude::*;

/// System responsible for creating new Rapier joints from the related `bevy_rapier` components.
pub fn init_joints(
    mut commands: Commands,
    mut context: ResMut<RapierContext>,
    impulse_joints: Query<(Entity, &ImpulseJoint), Without<RapierImpulseJointHandle>>,
    multibody_joints: Query<(Entity, &MultibodyJoint), Without<RapierMultibodyJointHandle>>,
    parent_query: Query<&Parent>,
) {
    let context = &mut *context;

    let impulse_joints = impulse_joints.iter();
    #[cfg(feature = "enhanced-determinism")]
    let mut impulse_joints: Vec<(Entity, &ImpulseJoint)> = impulse_joints.collect();
    #[cfg(feature = "enhanced-determinism")]
    impulse_joints.sort_unstable_by_key(|f| f.0);

    for (entity, joint) in impulse_joints {
        let mut target = None;
        let mut body_entity = entity;
        while target.is_none() {
            target = context.entity2body.get(&body_entity).copied();
            if let Ok(parent_entity) = parent_query.get(body_entity) {
                body_entity = parent_entity.get();
            } else {
                break;
            }
        }

        if let (Some(target), Some(source)) = (target, context.entity2body.get(&joint.parent)) {
            let handle = context.impulse_joints.insert(
                *source,
                target,
                joint.data.as_ref().into_rapier(),
                true,
            );
            commands
                .entity(entity)
                .insert(RapierImpulseJointHandle(handle));
            context.entity2impulse_joint.insert(entity, handle);
        }
    }

    let multibody_joints = multibody_joints.iter();
    #[cfg(feature = "enhanced-determinism")]
    let mut multibody_joints: Vec<(Entity, &MultibodyJoint)> = multibody_joints.collect();
    #[cfg(feature = "enhanced-determinism")]
    multibody_joints.sort_unstable_by_key(|f| f.0);

    for (entity, joint) in multibody_joints {
        let target = context.entity2body.get(&entity);

        if let (Some(target), Some(source)) = (target, context.entity2body.get(&joint.parent)) {
            if let Some(handle) = context.multibody_joints.insert(
                *source,
                *target,
                joint.data.as_ref().into_rapier(),
                true,
            ) {
                commands
                    .entity(entity)
                    .insert(RapierMultibodyJointHandle(handle));
                context.entity2multibody_joint.insert(entity, handle);
            } else {
                error!("Failed to create multibody joint: loop detected.")
            }
        }
    }
}

/// System responsible for applying changes the user made to a joint component.
pub fn apply_joint_user_changes(
    mut context: ResMut<RapierContext>,
    changed_impulse_joints: Query<
        (&RapierImpulseJointHandle, &ImpulseJoint),
        Changed<ImpulseJoint>,
    >,
    changed_multibody_joints: Query<
        (&RapierMultibodyJointHandle, &MultibodyJoint),
        Changed<MultibodyJoint>,
    >,
) {
    // TODO: right now, we only support propagating changes made to the joint data.
    //       Re-parenting the joint isn’t supported yet.
    for (handle, changed_joint) in changed_impulse_joints.iter() {
        if let Some(joint) = context.impulse_joints.get_mut(handle.0) {
            joint.data = changed_joint.data.as_ref().into_rapier();
        }
    }

    for (handle, changed_joint) in changed_multibody_joints.iter() {
        // TODO: not sure this will always work properly, e.g., if the number of Dofs is changed.
        if let Some((mb, link_id)) = context.multibody_joints.get_mut(handle.0) {
            if let Some(link) = mb.link_mut(link_id) {
                link.joint.data = changed_joint.data.as_ref().into_rapier();
            }
        }
    }
}
