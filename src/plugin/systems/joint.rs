use crate::dynamics::ImpulseJoint;
use crate::dynamics::MultibodyJoint;
use crate::dynamics::RapierImpulseJointHandle;
use crate::dynamics::RapierMultibodyJointHandle;
use crate::plugin::context::systemparams::RAPIER_CONTEXT_EXPECT_ERROR;
use crate::plugin::context::DefaultRapierContext;
use crate::plugin::context::RapierContextEntityLink;
use crate::plugin::context::RapierContextJoints;
use crate::plugin::context::RapierRigidBodySet;
use bevy::prelude::*;

/// System responsible for creating new Rapier joints from the related `bevy_rapier` components.
pub fn init_joints(
    mut commands: Commands,
    mut context_access: Query<(&RapierRigidBodySet, &mut RapierContextJoints)>,
    default_context_access: Query<Entity, With<DefaultRapierContext>>,
    impulse_joints: Query<
        (Entity, Option<&RapierContextEntityLink>, &ImpulseJoint),
        Without<RapierImpulseJointHandle>,
    >,
    multibody_joints: Query<
        (Entity, Option<&RapierContextEntityLink>, &MultibodyJoint),
        Without<RapierMultibodyJointHandle>,
    >,
    child_of_query: Query<&ChildOf>,
) {
    for (entity, entity_context_link, joint) in impulse_joints.iter() {
        // Get rapier context from RapierContextEntityLink or insert its default value.
        let context_entity = entity_context_link.map_or_else(
            || {
                let context_entity = default_context_access.single().ok()?;
                commands
                    .entity(entity)
                    .insert(RapierContextEntityLink(context_entity));
                Some(context_entity)
            },
            |link| Some(link.0),
        );
        let Some(context_entity) = context_entity else {
            continue;
        };

        let Ok(rigidbody_set_joints) = context_access.get_mut(context_entity) else {
            log::error!("Could not find entity {context_entity} with rapier context while initializing {entity}");
            continue;
        };
        let rigidbody_set = rigidbody_set_joints.0;
        let mut target = None;
        let mut body_entity = entity;
        while target.is_none() {
            target = rigidbody_set.entity2body.get(&body_entity).copied();
            if let Ok(child_of) = child_of_query.get(body_entity) {
                body_entity = child_of.parent;
            } else {
                break;
            }
        }
        let joints = rigidbody_set_joints.1.into_inner();

        if let (Some(target), Some(source)) = (target, rigidbody_set.entity2body.get(&joint.parent))
        {
            let handle = joints.impulse_joints.insert(
                *source,
                target,
                joint.data.as_ref().into_rapier(),
                true,
            );
            commands
                .entity(entity)
                .insert(RapierImpulseJointHandle(handle));
            joints.entity2impulse_joint.insert(entity, handle);
        }
    }

    for (entity, entity_context_link, joint) in multibody_joints.iter() {
        // Get rapier context from RapierContextEntityLink or insert its default value.
        let context_entity = entity_context_link.map_or_else(
            || {
                let context_entity = default_context_access.single().ok()?;
                commands
                    .entity(entity)
                    .insert(RapierContextEntityLink(context_entity));
                Some(context_entity)
            },
            |link| Some(link.0),
        );
        let Some(context_entity) = context_entity else {
            continue;
        };

        let Ok(context_joints) = context_access.get_mut(context_entity) else {
            log::error!("Could not find entity {context_entity} with rapier context while initializing {entity}");
            continue;
        };
        let context = context_joints.0;
        let target = context.entity2body.get(&entity);
        let joints = context_joints.1.into_inner();

        if let (Some(target), Some(source)) = (target, context.entity2body.get(&joint.parent)) {
            if let Some(handle) = joints.multibody_joints.insert(
                *source,
                *target,
                joint.data.as_ref().into_rapier(),
                true,
            ) {
                commands
                    .entity(entity)
                    .insert(RapierMultibodyJointHandle(handle));
                joints.entity2multibody_joint.insert(entity, handle);
            } else {
                log::error!("Failed to create multibody joint: loop detected.")
            }
        }
    }
}

/// System responsible for applying changes the user made to a joint component.
pub fn apply_joint_user_changes(
    mut context: Query<&mut RapierContextJoints>,
    changed_impulse_joints: Query<
        (
            &RapierContextEntityLink,
            &RapierImpulseJointHandle,
            &ImpulseJoint,
        ),
        Changed<ImpulseJoint>,
    >,
    changed_multibody_joints: Query<
        (
            &RapierContextEntityLink,
            &RapierMultibodyJointHandle,
            &MultibodyJoint,
        ),
        Changed<MultibodyJoint>,
    >,
) {
    // TODO: right now, we only support propagating changes made to the joint data.
    //       Re-parenting the joint isn’t supported yet.
    for (link, handle, changed_joint) in changed_impulse_joints.iter() {
        let mut context = context.get_mut(link.0).expect(RAPIER_CONTEXT_EXPECT_ERROR);
        if let Some(joint) = context.impulse_joints.get_mut(handle.0, false) {
            joint.data = changed_joint.data.as_ref().into_rapier();
        }
    }

    for (link, handle, changed_joint) in changed_multibody_joints.iter() {
        let mut context = context.get_mut(link.0).expect(RAPIER_CONTEXT_EXPECT_ERROR);
        // TODO: not sure this will always work properly, e.g., if the number of Dofs is changed.
        if let Some((mb, link_id)) = context.multibody_joints.get_mut(handle.0) {
            if let Some(link) = mb.link_mut(link_id) {
                link.joint.data = changed_joint.data.as_ref().into_rapier();
            }
        }
    }
}
