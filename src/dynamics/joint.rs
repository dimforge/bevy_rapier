use crate::dynamics::GenericJoint;
use bevy::prelude::*;
use rapier::dynamics::{ImpulseJointHandle, MultibodyJointHandle};

#[derive(Copy, Clone, Debug, Component)]
pub struct RapierImpulseJointHandle(pub ImpulseJointHandle);

#[derive(Copy, Clone, Debug, Component)]
pub struct RapierMultibodyJointHandle(pub MultibodyJointHandle);

#[derive(Copy, Clone, Debug, PartialEq, Component)]
pub struct ImpulseJoint {
    pub parent: Entity,
    pub data: GenericJoint,
}

impl ImpulseJoint {
    pub fn new(parent: Entity, data: impl Into<GenericJoint>) -> Self {
        Self {
            parent,
            data: data.into(),
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Component)]
pub struct MultibodyJoint {
    pub parent: Entity,
    pub data: GenericJoint,
}

impl MultibodyJoint {
    pub fn new(parent: Entity, data: impl Into<GenericJoint>) -> Self {
        Self {
            parent,
            data: data.into(),
        }
    }
}
