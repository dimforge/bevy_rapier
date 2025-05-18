use crate::dynamics::ImpulseJoint;
use crate::dynamics::MultibodyJoint;
use crate::dynamics::RapierImpulseJointHandle;
use crate::dynamics::RapierMultibodyJointHandle;
use crate::dynamics::RapierRigidBodyHandle;
use crate::dynamics::RigidBody;
use crate::geometry::Collider;
use crate::geometry::ColliderDisabled;
use crate::geometry::RapierColliderHandle;
use crate::plugin::context::{
    RapierContextColliders, RapierContextJoints, RapierContextSimulation, RapierRigidBodySet,
};
use crate::prelude::MassModifiedEvent;
use crate::prelude::RigidBodyDisabled;
use crate::prelude::Sensor;
use bevy::ecs::query::QueryData;
use bevy::prelude::*;

/// System responsible for removing from Rapier the rigid-bodies/colliders/joints which had
/// their related `bevy_rapier` components removed by the user (through component removal or
/// despawn).
pub fn sync_removals(
    mut commands: Commands,
    mut context_writer: Query<(
        &mut RapierContextSimulation,
        &mut RapierContextColliders,
        &mut RapierContextJoints,
        &mut RapierRigidBodySet,
    )>,
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
        let Some(((mut context, mut context_colliders, mut joints, mut rigidbody_set), handle)) =
            find_context(&mut context_writer, |res| res.3.entity2body.remove(&entity))
        else {
            continue;
        };
        let context = &mut *context;
        let joints = &mut *joints;

        let _ = rigidbody_set.last_body_transform_set.remove(&handle);
        rigidbody_set.bodies.remove(
            handle,
            &mut context.islands,
            &mut context_colliders.colliders,
            &mut joints.impulse_joints,
            &mut joints.multibody_joints,
            false,
        );
    }

    for entity in orphan_bodies.iter() {
        if let Some(((mut context, mut context_colliders, mut joints, mut rigidbody_set), handle)) =
            find_context(&mut context_writer, |res| res.3.entity2body.remove(&entity))
        {
            let context = &mut *context;
            let joints = &mut *joints;
            let _ = rigidbody_set.last_body_transform_set.remove(&handle);
            rigidbody_set.bodies.remove(
                handle,
                &mut context.islands,
                &mut context_colliders.colliders,
                &mut joints.impulse_joints,
                &mut joints.multibody_joints,
                false,
            );
        }
        commands.entity(entity).remove::<RapierRigidBodyHandle>();
    }

    /*
     * Collider removal detection.
     */
    for entity in removed_colliders.read() {
        let Some(((mut context, mut context_colliders, _, mut rigidbody_set), handle)) =
            find_context(&mut context_writer, |res| {
                res.1.entity2collider.remove(&entity)
            })
        else {
            continue;
        };
        let context = &mut *context;
        if let Some(parent) = context_colliders.collider_parent(&rigidbody_set, entity) {
            mass_modified.write(parent.into());
        }

        context_colliders.colliders.remove(
            handle,
            &mut context.islands,
            &mut rigidbody_set.bodies,
            true,
        );
        context.deleted_colliders.insert(handle, entity);
    }

    for entity in orphan_colliders.iter() {
        if let Some(((mut context, mut context_colliders, _, mut rigidbody_set), handle)) =
            find_context(&mut context_writer, |res| {
                res.1.entity2collider.remove(&entity)
            })
        {
            let context = &mut *context;
            let context_colliders = &mut *context_colliders;
            if let Some(parent) = context_colliders.collider_parent(&rigidbody_set, entity) {
                mass_modified.write(parent.into());
            }

            context_colliders.colliders.remove(
                handle,
                &mut context.islands,
                &mut rigidbody_set.bodies,
                true,
            );
            context.deleted_colliders.insert(handle, entity);
        }
        commands.entity(entity).remove::<RapierColliderHandle>();
    }

    /*
     * Impulse joint removal detection.
     */
    for entity in removed_impulse_joints.read() {
        let Some(((_, _, mut joints, _), handle)) = find_context(&mut context_writer, |res| {
            res.2.entity2impulse_joint.remove(&entity)
        }) else {
            continue;
        };
        joints.impulse_joints.remove(handle, true);
    }

    for entity in orphan_impulse_joints.iter() {
        if let Some(((_, _, mut joints, _), handle)) = find_context(&mut context_writer, |res| {
            res.2.entity2impulse_joint.remove(&entity)
        }) {
            joints.impulse_joints.remove(handle, true);
        }
        commands.entity(entity).remove::<RapierImpulseJointHandle>();
    }

    /*
     * Multibody joint removal detection.
     */
    for entity in removed_multibody_joints.read() {
        let Some(((_, _, mut joints, _), handle)) = find_context(&mut context_writer, |res| {
            res.2.entity2multibody_joint.remove(&entity)
        }) else {
            continue;
        };
        joints.multibody_joints.remove(handle, true);
    }

    for entity in orphan_multibody_joints.iter() {
        if let Some(((_, _, mut joints, _), handle)) = find_context(&mut context_writer, |res| {
            res.2.entity2multibody_joint.remove(&entity)
        }) {
            joints.multibody_joints.remove(handle, true);
        }
        commands
            .entity(entity)
            .remove::<RapierMultibodyJointHandle>();
    }

    /*
     * Marker components removal detection.
     */
    for entity in removed_sensors.read() {
        if let Some((mut context, handle)) = find_context(&mut context_writer, |context| {
            context.1.entity2collider.get(&entity).copied()
        }) {
            if let Some(co) = context.1.colliders.get_mut(handle) {
                co.set_sensor(false);
            }
        }
    }

    for entity in removed_colliders_disabled.read() {
        if let Some((mut context, handle)) = find_context(&mut context_writer, |context| {
            context.1.entity2collider.get(&entity).copied()
        }) {
            if let Some(co) = context.1.colliders.get_mut(handle) {
                co.set_enabled(true);
            }
        }
    }

    for entity in removed_rigid_body_disabled.read() {
        if let Some(((_, _, _, mut rigidbody_set), handle)) =
            find_context(&mut context_writer, |res| {
                res.3.entity2body.get(&entity).copied()
            })
        {
            if let Some(rb) = rigidbody_set.bodies.get_mut(handle) {
                rb.set_enabled(true);
            }
        }
    }

    // TODO: what about removing forces?
}

fn find_context<'a, TReturn, TQueryParams: QueryData>(
    context_writer: &'a mut Query<TQueryParams>,
    item_finder: impl Fn(&mut TQueryParams::Item<'_>) -> Option<TReturn>,
) -> Option<(TQueryParams::Item<'a>, TReturn)> {
    let ret: Option<(TQueryParams::Item<'_>, TReturn)> = context_writer
        .iter_mut()
        .find_map(|mut context| item_finder(&mut context).map(|handle| (context, handle)));
    ret
}
