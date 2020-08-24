use bevy::prelude::*;
use rapier::dynamics::{JointHandle, JointParams, RigidBodyHandle};
use rapier::geometry::ColliderHandle;

/// A component representing a rigid-body that is being handled by
/// a Rapier physics World.
pub struct RigidBodyHandleComponent(RigidBodyHandle);

impl From<RigidBodyHandle> for RigidBodyHandleComponent {
    fn from(handle: RigidBodyHandle) -> Self {
        Self(handle)
    }
}

impl RigidBodyHandleComponent {
    /// The handle of thi rigid-body managed by a Rapier physics World.
    ///
    /// This can be passed to a `RigidBodySet` to retrieve a reference to a Rapier rigid-body.
    pub fn handle(&self) -> RigidBodyHandle {
        self.0
    }
}

/// A component representing a collider that is being handled by
/// a Rapier physics World.
pub struct ColliderHandleComponent(ColliderHandle);

impl From<ColliderHandle> for ColliderHandleComponent {
    fn from(handle: ColliderHandle) -> Self {
        Self(handle)
    }
}

impl ColliderHandleComponent {
    /// The handle of the collider managed by a Rapier physics World.
    ///
    /// This can be passed to a `ColliderSet` to retrieve a reference to a Rapier rigid-body.
    pub fn handle(&self) -> ColliderHandle {
        self.0
    }
}

pub struct JointHandleComponent {
    handle: JointHandle,
    entity1: Entity,
    entity2: Entity,
}

impl JointHandleComponent {
    pub(crate) fn new(handle: JointHandle, entity1: Entity, entity2: Entity) -> Self {
        Self {
            handle,
            entity1,
            entity2,
        }
    }

    pub fn handle(&self) -> JointHandle {
        self.handle
    }

    pub fn entity1(&self) -> Entity {
        self.entity1
    }

    pub fn entity2(&self) -> Entity {
        self.entity2
    }
}

pub struct JointBuilderComponent {
    pub(crate) params: JointParams,
    pub(crate) entity1: Entity,
    pub(crate) entity2: Entity,
}

impl JointBuilderComponent {
    pub fn new<J>(joint: J, entity1: Entity, entity2: Entity) -> Self
    where
        J: Into<JointParams>,
    {
        JointBuilderComponent {
            params: joint.into(),
            entity1,
            entity2,
        }
    }
}
