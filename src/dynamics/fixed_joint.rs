use crate::dynamics::{GenericJoint, GenericJointBuilder};
use crate::math::{AsPrecise, Rot, Vect};
use rapier::dynamics::JointAxesMask;

#[derive(Copy, Clone, Debug, PartialEq)]
#[repr(transparent)]
/// A fixed joint, locks all relative motion between two bodies.
pub struct FixedJoint {
    data: GenericJoint,
}

impl Default for FixedJoint {
    fn default() -> Self {
        FixedJoint::new()
    }
}

impl FixedJoint {
    /// Creates a new fixed joint.
    #[must_use]
    pub fn new() -> Self {
        let data = GenericJointBuilder::new(JointAxesMask::LOCKED_FIXED_AXES).build();
        Self { data }
    }

    /// Are contacts between the attached rigid-bodies enabled?
    pub fn contacts_enabled(&self) -> bool {
        self.data.contacts_enabled()
    }

    /// Sets whether contacts between the attached rigid-bodies are enabled.
    pub fn set_contacts_enabled(&mut self, enabled: bool) -> &mut Self {
        self.data.set_contacts_enabled(enabled);
        self
    }

    /// The joint’s basis, expressed in the first rigid-body’s local-space.
    #[must_use]
    pub fn local_basis1(&self) -> Rot {
        self.data.local_basis1()
    }

    /// Sets the joint’s basis, expressed in the first rigid-body’s local-space.
    pub fn set_local_basis1(&mut self, local_basis: impl AsPrecise<Out = Rot>) -> &mut Self {
        self.data.set_local_basis1(local_basis.as_precise());
        self
    }

    /// The joint’s basis, expressed in the second rigid-body’s local-space.
    #[must_use]
    pub fn local_basis2(&self) -> Rot {
        self.data.local_basis2()
    }

    /// Sets joint’s basis, expressed in the second rigid-body’s local-space.
    pub fn set_local_basis2(&mut self, local_basis: impl AsPrecise<Out = Rot>) -> &mut Self {
        self.data.set_local_basis2(local_basis.as_precise());
        self
    }

    /// The joint’s anchor, expressed in the local-space of the first rigid-body.
    #[must_use]
    pub fn local_anchor1(&self) -> Vect {
        self.data.local_anchor1()
    }

    /// Sets the joint’s anchor, expressed in the local-space of the first rigid-body.
    pub fn set_local_anchor1(&mut self, anchor1: impl AsPrecise<Out = Vect>) -> &mut Self {
        self.data.set_local_anchor1(anchor1.as_precise());
        self
    }

    /// The joint’s anchor, expressed in the local-space of the second rigid-body.
    #[must_use]
    pub fn local_anchor2(&self) -> Vect {
        self.data.local_anchor2()
    }

    /// Sets the joint’s anchor, expressed in the local-space of the second rigid-body.
    pub fn set_local_anchor2(&mut self, anchor2: impl AsPrecise<Out = Vect>) -> &mut Self {
        self.data.set_local_anchor2(anchor2.as_precise());
        self
    }
}

impl From<FixedJoint> for GenericJoint {
    fn from(joint: FixedJoint) -> GenericJoint {
        joint.data
    }
}

/// Create fixed joints using the builder pattern.
#[derive(Copy, Clone, Debug, PartialEq, Default)]
pub struct FixedJointBuilder(FixedJoint);

impl FixedJointBuilder {
    /// Creates a new builder for fixed joints.
    pub fn new() -> Self {
        Self(FixedJoint::new())
    }

    /// Sets the joint’s basis, expressed in the first rigid-body’s local-space.
    #[must_use]
    pub fn local_basis1(mut self, local_basis: impl AsPrecise<Out = Rot>) -> Self {
        self.0.set_local_basis1(local_basis.as_precise());
        self
    }

    /// Sets joint’s basis, expressed in the second rigid-body’s local-space.
    #[must_use]
    pub fn local_basis2(mut self, local_basis: impl AsPrecise<Out = Rot>) -> Self {
        self.0.set_local_basis2(local_basis.as_precise());
        self
    }

    /// Sets the joint’s anchor, expressed in the local-space of the first rigid-body.
    #[must_use]
    pub fn local_anchor1(mut self, anchor1: impl AsPrecise<Out = Vect>) -> Self {
        self.0.set_local_anchor1(anchor1.as_precise());
        self
    }

    /// Sets the joint’s anchor, expressed in the local-space of the second rigid-body.
    #[must_use]
    pub fn local_anchor2(mut self, anchor2: impl AsPrecise<Out = Vect>) -> Self {
        self.0.set_local_anchor2(anchor2.as_precise());
        self
    }

    /// Build the fixed joint.
    #[must_use]
    pub fn build(self) -> FixedJoint {
        self.0
    }
}

impl From<FixedJointBuilder> for GenericJoint {
    fn from(joint: FixedJointBuilder) -> GenericJoint {
        joint.0.into()
    }
}
