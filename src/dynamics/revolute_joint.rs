//use crate::dynamics::{GenericJoint, GenericJointBuilder};
use crate::math::Vect;
use rapier::prelude::{GenericJoint, RevoluteJoint as RapierRevoluteJoint, RevoluteJointBuilder};

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Copy, Clone, Debug, PartialEq)]
#[repr(transparent)]
/// Wrapper for rapier's [`RapierRevoluteJoint`], allowing to add more methods compatible with glam.
///
/// For more info see [`RapierRevoluteJoint`].
pub struct RevoluteJoint {
    /// Inner data from rapier.
    pub data: RapierRevoluteJoint,
}

#[cfg(feature = "dim2")]
impl Default for RevoluteJoint {
    fn default() -> Self {
        Self::new_glam()
    }
}

impl RevoluteJoint {
    /// Creates a new revolute joint allowing only relative rotations.
    #[cfg(feature = "dim2")]
    pub fn new_glam() -> Self {
        let data = RevoluteJointBuilder::new();
        Self { data: data.build() }
    }

    /// Creates a new revolute joint allowing only relative rotations along the specified axis.
    ///
    /// This axis is expressed in the local-space of both rigid-bodies.
    #[cfg(feature = "dim3")]
    pub fn new_glam(axis: Vect) -> Self {
        use rapier::math::UnitVector;
        let vector = UnitVector::<rapier::math::Real>::new_normalize(axis.into());
        let data = RevoluteJointBuilder::new(vector).build();
        Self { data }
    }

    /// The joint’s anchor, expressed in the local-space of the first rigid-body.
    #[must_use]
    pub fn local_anchor1_glam(&self) -> Vect {
        self.data.local_anchor1().into()
    }

    /// Sets the joint’s anchor, expressed in the local-space of the first rigid-body.
    pub fn set_local_anchor1_glam(&mut self, anchor1: Vect) -> &mut Self {
        self.data.set_local_anchor1(anchor1.into());
        self
    }

    /// The joint’s anchor, expressed in the local-space of the second rigid-body.
    #[must_use]
    pub fn local_anchor2_glam(&self) -> Vect {
        self.data.local_anchor2().into()
    }

    /// Sets the joint’s anchor, expressed in the local-space of the second rigid-body.
    pub fn set_local_anchor2_glam(&mut self, anchor2: Vect) -> &mut Self {
        self.data.set_local_anchor2(anchor2.into());
        self
    }
}

impl From<RevoluteJoint> for GenericJoint {
    fn from(joint: RevoluteJoint) -> GenericJoint {
        joint.data.data
    }
}

/// A trait to extend [`rapier::prelude::RevoluteJointBuilder`] with glam types.
pub trait RevoluteJointBuilderGlam {
    /// Creates a new revolute joint allowing only relative rotations.
    #[cfg(feature = "dim2")]
    fn new_glam() -> Self;

    /// Creates a new revolute joint builder, allowing only relative rotations along the specified axis.
    ///
    /// This axis is expressed in the local-space of both rigid-bodies.
    #[cfg(feature = "dim3")]
    fn new_glam(axis: Vect) -> Self;

    /// Sets the joint’s anchor, expressed in the local-space of the first rigid-body.
    fn local_anchor1_glam(self: Self, anchor1: Vect) -> Self;

    /// Sets the joint’s anchor, expressed in the local-space of the second rigid-body.
    fn local_anchor2_glam(self: Self, anchor2: Vect) -> Self;
}

impl RevoluteJointBuilderGlam for RevoluteJointBuilder {
    #[cfg(feature = "dim2")]
    fn new_glam() -> Self {
        Self(RapierRevoluteJoint::new())
    }

    #[cfg(feature = "dim3")]
    fn new_glam(axis: Vect) -> Self {
        use rapier::math::UnitVector;
        let vector = UnitVector::<rapier::math::Real>::new_normalize(axis.into());
        Self(RapierRevoluteJoint::new(vector))
    }

    #[must_use]
    fn local_anchor1_glam(self, anchor1: Vect) -> Self {
        RevoluteJointBuilder::local_anchor1(self, anchor1.into())
    }

    #[must_use]
    fn local_anchor2_glam(self, anchor2: Vect) -> Self {
        RevoluteJointBuilder::local_anchor2(self, anchor2.into())
    }
}
