use crate::dynamics::ImpulseJoint;
use crate::dynamics::MultibodyJoint;
use crate::dynamics::RapierImpulseJointHandle;
use crate::dynamics::RapierMultibodyJointHandle;
use crate::dynamics::RapierRigidBodyHandle;
use crate::dynamics::RigidBody;
use crate::geometry::Collider;
use crate::geometry::ColliderDisabled;
use crate::geometry::RapierColliderHandle;
use crate::plugin::context::systemparams::RapierContextAccessMut;
use crate::plugin::RapierContext;
use crate::prelude::MassModifiedEvent;
use crate::prelude::RigidBodyDisabled;
use crate::prelude::Sensor;
use bevy::prelude::*;

/// System responsible for removing from Rapier the rigid-bodies/colliders/joints which had
/// their related `bevy_rapier` components removed by the user (through component removal or
/// despawn).
pub fn sync_removals(
    mut commands: Commands,
    mut context_accessor: RapierContextAccessMut,
    mut removed_bodies: RemovedComponents<RapierRigidBodyHandle>,
    mut removed_colliders: RemovedComponents<RapierColliderHandle>,
    mut removed_impulse_joints: RemovedComponents<RapierImpulseJointHandle>,
    mut removed_multibody_joints: RemovedComponents<RapierMultibodyJointHandle>,
    orphan_bodies: Query<Entity, (With<RapierRigidBodyHandle>, Without<RigidBody>)>,
    orphan_colliders: Query<Entity, (With<RapierColliderHandle>, Without<Collider>)>,
    orphan_impulse_joints: Query<Entity, (With<RapierImpulseJointHandle>, Without<ImpulseJoint>)>,
    orphan_multibody_joints: Query<
        Entity,
        (With<RapierMultibodyJointHandle>, Without<MultibodyJoint>),
    >,

    mut removed_sensors: RemovedComponents<Sensor>,
    mut removed_rigid_body_disabled: RemovedComponents<RigidBodyDisabled>,
    mut removed_colliders_disabled: RemovedComponents<ColliderDisabled>,

    mut mass_modified: EventWriter<MassModifiedEvent>,
) {
    /*
     * Rigid-bodies removal detection.
     */
    for entity in removed_bodies.read() {
        let Some((mut context, handle)) = find_context(&mut context_accessor, |context| {
            context.entity2body.remove(&entity)
        }) else {
            continue;
        };
        let context = &mut *context;

        let _ = context.last_body_transform_set.remove(&handle);
        context.bodies.remove(
            handle,
            &mut context.islands,
            &mut context.colliders,
            &mut context.impulse_joints,
            &mut context.multibody_joints,
            false,
        );
    }

    for entity in orphan_bodies.iter() {
        if let Some((mut context, handle)) = find_context(&mut context_accessor, |context| {
            context.entity2body.remove(&entity)
        }) {
            let context = &mut *context;
            let _ = context.last_body_transform_set.remove(&handle);
            context.bodies.remove(
                handle,
                &mut context.islands,
                &mut context.colliders,
                &mut context.impulse_joints,
                &mut context.multibody_joints,
                false,
            );
        }
        commands.entity(entity).remove::<RapierRigidBodyHandle>();
    }

    /*
     * Collider removal detection.
     */
    for entity in removed_colliders.read() {
        let Some((mut context, handle)) = find_context(&mut context_accessor, |context| {
            context.entity2collider.remove(&entity)
        }) else {
            continue;
        };
        let context = &mut *context;
        if let Some(parent) = context.collider_parent(entity) {
            mass_modified.send(parent.into());
        }

        context
            .colliders
            .remove(handle, &mut context.islands, &mut context.bodies, true);
        context.deleted_colliders.insert(handle, entity);
    }

    for entity in orphan_colliders.iter() {
        if let Some((mut context, handle)) = find_context(&mut context_accessor, |context| {
            context.entity2collider.remove(&entity)
        }) {
            let context = &mut *context;
            if let Some(parent) = context.collider_parent(entity) {
                mass_modified.send(parent.into());
            }

            context
                .colliders
                .remove(handle, &mut context.islands, &mut context.bodies, true);
            context.deleted_colliders.insert(handle, entity);
        }
        commands.entity(entity).remove::<RapierColliderHandle>();
    }

    /*
     * Impulse joint removal detection.
     */
    for entity in removed_impulse_joints.read() {
        let Some((mut context, handle)) = find_context(&mut context_accessor, |context| {
            context.entity2impulse_joint.remove(&entity)
        }) else {
            continue;
        };
        let context = &mut *context;
        context.impulse_joints.remove(handle, true);
    }

    for entity in orphan_impulse_joints.iter() {
        if let Some((mut context, handle)) = find_context(&mut context_accessor, |context| {
            context.entity2impulse_joint.remove(&entity)
        }) {
            let context = &mut *context;
            context.impulse_joints.remove(handle, true);
        }
        commands.entity(entity).remove::<RapierImpulseJointHandle>();
    }

    /*
     * Multibody joint removal detection.
     */
    for entity in removed_multibody_joints.read() {
        let Some((mut context, handle)) = find_context(&mut context_accessor, |context| {
            context.entity2multibody_joint.remove(&entity)
        }) else {
            continue;
        };
        let context = &mut *context;
        context.multibody_joints.remove(handle, true);
    }

    for entity in orphan_multibody_joints.iter() {
        if let Some((mut context, handle)) = find_context(&mut context_accessor, |context| {
            context.entity2multibody_joint.remove(&entity)
        }) {
            let context = &mut *context;
            context.multibody_joints.remove(handle, true);
        }
        commands
            .entity(entity)
            .remove::<RapierMultibodyJointHandle>();
    }

    /*
     * Marker components removal detection.
     */
    for entity in removed_sensors.read() {
        if let Some((mut context, handle)) = find_context(&mut context_accessor, |context| {
            context.entity2collider.get(&entity).copied()
        }) {
            if let Some(co) = context.colliders.get_mut(handle) {
                co.set_sensor(false);
            }
        }
    }

    for entity in removed_colliders_disabled.read() {
        if let Some((mut context, handle)) = find_context(&mut context_accessor, |context| {
            context.entity2collider.get(&entity).copied()
        }) {
            if let Some(co) = context.colliders.get_mut(handle) {
                co.set_enabled(true);
            }
        }
    }

    for entity in removed_rigid_body_disabled.read() {
        if let Some((mut context, handle)) = find_context(&mut context_accessor, |context| {
            context.entity2body.get(&entity).copied()
        }) {
            if let Some(rb) = context.bodies.get_mut(handle) {
                rb.set_enabled(true);
            }
        }
    }

    // TODO: what about removing forces?
}

fn find_context<'a, T>(
    context_accessor: &'a mut RapierContextAccessMut,
    item_finder: impl Fn(&mut RapierContext) -> Option<T>,
) -> Option<(Mut<'a, RapierContext>, T)> {
    context_accessor
        .rapier_context
        .iter_mut()
        .find_map(|mut context| item_finder(&mut context).map(|handle| (context, handle)))
}
