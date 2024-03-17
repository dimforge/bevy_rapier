use crate::dynamics::{
    ImpulseJoint, ImpulseJointCreated, ImpulseJointHandle, MultibodyJoint, MultibodyJointCreated,
    MultibodyJointHandle, RigidBody, RigidBodyCreated, RigidBodyHandle,
};
use crate::geometry::{Collider, ColliderDisabled, ColliderHandle, Sensor};
use crate::plugin::RapierContext;
use crate::prelude::{ColliderCreated, MassModifiedEvent, RigidBodyDisabled};
use bevy::prelude::*;

#[cfg(feature = "dim2")]
use bevy::math::Vec3Swizzles;

/// System responsible for removing from Rapier the rigid-bodies/colliders/joints which had
/// their related `bevy_rapier` components removed by the user (through component removal or
/// despawn).
pub fn sync_removals(
    mut commands: Commands,
    mut context: ResMut<RapierContext>,
    mut removed_bodies: RemovedComponents<RigidBodyCreated>,
    mut removed_colliders: RemovedComponents<ColliderCreated>,
    mut removed_impulse_joints: RemovedComponents<ImpulseJointCreated>,
    mut removed_multibody_joints: RemovedComponents<MultibodyJointCreated>,
    orphan_bodies: Query<Entity, (With<RigidBodyCreated>, Without<RigidBody>)>,
    orphan_colliders: Query<Entity, (With<ColliderCreated>, Without<Collider>)>,
    orphan_impulse_joints: Query<Entity, (With<ImpulseJointCreated>, Without<ImpulseJoint>)>,
    orphan_multibody_joints: Query<Entity, (With<MultibodyJointCreated>, Without<MultibodyJoint>)>,
    mut removed_sensors: RemovedComponents<Sensor>,
    mut removed_rigid_body_disabled: RemovedComponents<RigidBodyDisabled>,
    mut removed_colliders_disabled: RemovedComponents<ColliderDisabled>,
    mut mass_modified: EventWriter<MassModifiedEvent>,
) {
    /*
     * Rigid-bodies removal detection.
     */
    let context = &mut *context;
    for entity in removed_bodies.read() {
        let _ = context.last_body_transform_set.remove(&entity);
        context.bodies.remove(
            entity,
            &mut context.islands,
            &mut context.colliders,
            &mut context.impulse_joints,
            &mut context.multibody_joints,
            false,
        );
    }

    let context = &mut *context;
    for entity in orphan_bodies.iter() {
        let _ = context.last_body_transform_set.remove(&entity);
        context.bodies.remove(
            entity,
            &mut context.islands,
            &mut context.colliders,
            &mut context.impulse_joints,
            &mut context.multibody_joints,
            false,
        );
        commands.entity(entity).remove::<RigidBodyCreated>();
    }

    /*
     * Collider removal detection.
     */
    for entity in removed_colliders.read() {
        if let Some(parent) = context.collider_parent(entity) {
            mass_modified.send(parent.into());
        }

        context
            .colliders
            .remove(entity, &mut context.islands, &mut context.bodies, true);
        context.deleted_colliders.insert(entity);
    }

    for entity in orphan_colliders.iter() {
        if let Some(parent) = context.collider_parent(entity) {
            mass_modified.send(parent.into());
        }

        context
            .colliders
            .remove(entity, &mut context.islands, &mut context.bodies, true);
        context.deleted_colliders.insert(entity);

        commands.entity(entity).remove::<ColliderCreated>();
    }

    /*
     * Impulse joint removal detection.
     */
    for entity in removed_impulse_joints.read() {
        context.impulse_joints.remove(entity, true);
    }

    for entity in orphan_impulse_joints.iter() {
        context.impulse_joints.remove(entity, true);
        commands.entity(entity).remove::<ImpulseJointCreated>();
    }

    /*
     * Multibody joint removal detection.
     */
    for entity in removed_multibody_joints.read() {
        context.multibody_joints.remove(entity, true);
    }

    for entity in orphan_multibody_joints.iter() {
        context.multibody_joints.remove(entity, true);
        commands.entity(entity).remove::<MultibodyJointCreated>();
    }

    /*
     * Marker components removal detection.
     */
    for entity in removed_sensors.read() {
        if let Some(co) = context.colliders.get_mut(entity) {
            co.set_sensor(false);
        }
    }

    for entity in removed_colliders_disabled.read() {
        if let Some(co) = context.colliders.get_mut(entity) {
            co.set_enabled(true);
        }
    }

    for entity in removed_rigid_body_disabled.read() {
        if let Some(rb) = context.bodies.get_mut(entity) {
            rb.set_enabled(true);
        }
    }

    // TODO: what about removing forces?
}
