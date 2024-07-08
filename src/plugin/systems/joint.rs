use crate::dynamics::ImpulseJoint;
use crate::dynamics::MultibodyJoint;
use crate::dynamics::RapierImpulseJointHandle;
use crate::dynamics::RapierMultibodyJointHandle;
use crate::plugin::context::systemparams::get_single_context;
use crate::plugin::context::RapierContextEntityLink;
use crate::plugin::DefaultRapierContext;
use crate::plugin::RapierContextAccessMut;
use bevy::prelude::*;

/// System responsible for creating new Rapier joints from the related `bevy_rapier` components.
pub fn init_joints(
    mut commands: Commands,
    mut context_access: RapierContextAccessMut,
    default_context: Query<Entity, With<DefaultRapierContext>>,
    impulse_joints: Query<
        (Entity, Option<&RapierContextEntityLink>, &ImpulseJoint),
        Without<RapierImpulseJointHandle>,
    >,
    multibody_joints: Query<
        (Entity, Option<&RapierContextEntityLink>, &MultibodyJoint),
        Without<RapierMultibodyJointHandle>,
    >,
    parent_query: Query<&Parent>,
) {
    for (entity, entity_context_link, joint) in impulse_joints.iter() {
        let context_entity = entity_context_link.map_or_else(
            || {
                let context_entity = get_single_context(&default_context);
                commands
                    .entity(entity)
                    .insert(RapierContextEntityLink(context_entity));
                context_entity
            },
            |link| link.0,
        );
        let context = context_access
            .context_from_entity(context_entity)
            .into_inner();
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

    for (entity, entity_context_link, joint) in multibody_joints.iter() {
        let context_entity = entity_context_link.map_or_else(
            || {
                let context_entity = get_single_context(&default_context);
                commands
                    .entity(entity)
                    .insert(RapierContextEntityLink(context_entity));
                context_entity
            },
            |link| link.0,
        );
        let context = context_access
            .context_from_entity(context_entity)
            .into_inner();
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
    mut context: RapierContextAccessMut,
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
        let context = context.context(link).into_inner();
        if let Some(joint) = context.impulse_joints.get_mut(handle.0) {
            joint.data = changed_joint.data.as_ref().into_rapier();
        }
    }

    for (link, handle, changed_joint) in changed_multibody_joints.iter() {
        let context = context.context(link).into_inner();
        // TODO: not sure this will always work properly, e.g., if the number of Dofs is changed.
        if let Some((mb, link_id)) = context.multibody_joints.get_mut(handle.0) {
            if let Some(link) = mb.link_mut(link_id) {
                link.joint.data = changed_joint.data.as_ref().into_rapier();
            }
        }
    }
}
