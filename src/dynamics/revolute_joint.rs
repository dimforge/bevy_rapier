use crate::dynamics::{GenericJoint, GenericJointBuilder};
use crate::math::{AsPrecise, Real, Vect};
use rapier::dynamics::{JointAxesMask, JointAxis, JointLimits, JointMotor, MotorModel};

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Copy, Clone, Debug, PartialEq)]
#[repr(transparent)]
/// A revolute joint, locks all relative motion except for rotation along the joint’s principal axis.
pub struct RevoluteJoint {
    data: GenericJoint,
}

#[cfg(feature = "dim2")]
impl Default for RevoluteJoint {
    fn default() -> Self {
        Self::new()
    }
}

impl RevoluteJoint {
    /// Creates a new revolute joint allowing only relative rotations.
    #[cfg(feature = "dim2")]
    pub fn new() -> Self {
        let data = GenericJointBuilder::new(JointAxesMask::LOCKED_REVOLUTE_AXES);
        Self { data: data.build() }
    }

    /// Creates a new revolute joint allowing only relative rotations along the specified axis.
    ///
    /// This axis is expressed in the local-space of both rigid-bodies.
    #[cfg(feature = "dim3")]
    pub fn new(axis: impl AsPrecise<Out = Vect>) -> Self {
        let axis = axis.as_precise();
        let data = GenericJointBuilder::new(JointAxesMask::LOCKED_REVOLUTE_AXES)
            .local_axis1(axis)
            .local_axis2(axis)
            .build();
        Self { data }
    }

    /// The underlying generic joint.
    pub fn data(&self) -> &GenericJoint {
        &self.data
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

    /// The motor affecting the joint’s rotational degree of freedom.
    #[must_use]
    pub fn motor(&self) -> Option<&JointMotor> {
        self.data.motor(JointAxis::AngX)
    }

    /// Set the spring-like model used by the motor to reach the desired target velocity and position.
    pub fn set_motor_model(&mut self, model: MotorModel) -> &mut Self {
        self.data.set_motor_model(JointAxis::AngX, model);
        self
    }

    /// Sets the target velocity this motor needs to reach.
    pub fn set_motor_velocity(&mut self, target_vel: Real, factor: Real) -> &mut Self {
        self.data
            .set_motor_velocity(JointAxis::AngX, target_vel, factor);
        self
    }

    /// Sets the target angle this motor needs to reach.
    pub fn set_motor_position(
        &mut self,
        target_pos: Real,
        stiffness: Real,
        damping: Real,
    ) -> &mut Self {
        self.data
            .set_motor_position(JointAxis::AngX, target_pos, stiffness, damping);
        self
    }

    /// Configure both the target angle and target velocity of the motor.
    pub fn set_motor(
        &mut self,
        target_pos: Real,
        target_vel: Real,
        stiffness: Real,
        damping: Real,
    ) -> &mut Self {
        self.data
            .set_motor(JointAxis::AngX, target_pos, target_vel, stiffness, damping);
        self
    }

    /// Sets the maximum force the motor can deliver.
    pub fn set_motor_max_force(&mut self, max_force: Real) -> &mut Self {
        self.data.set_motor_max_force(JointAxis::AngX, max_force);
        self
    }

    /// The limit angle attached bodies can translate along the joint’s principal axis.
    #[must_use]
    pub fn limits(&self) -> Option<&JointLimits<Real>> {
        self.data.limits(JointAxis::AngX)
    }

    /// Sets the `[min,max]` limit angle attached bodies can translate along the joint’s principal axis.
    pub fn set_limits(&mut self, limits: [Real; 2]) -> &mut Self {
        self.data.set_limits(JointAxis::AngX, limits);
        self
    }
}

impl From<RevoluteJoint> for GenericJoint {
    fn from(joint: RevoluteJoint) -> GenericJoint {
        joint.data
    }
}

/// Create revolute joints using the builder pattern.
///
/// A revolute joint locks all relative motion except for rotations along the joint’s principal axis.
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct RevoluteJointBuilder(RevoluteJoint);

#[cfg(feature = "dim2")]
impl Default for RevoluteJointBuilder {
    fn default() -> Self {
        Self::new()
    }
}

impl RevoluteJointBuilder {
    /// Creates a new revolute joint builder.
    #[cfg(feature = "dim2")]
    pub fn new() -> Self {
        Self(RevoluteJoint::new())
    }

    /// Creates a new revolute joint builder, allowing only relative rotations along the specified axis.
    ///
    /// This axis is expressed in the local-space of both rigid-bodies.
    #[cfg(feature = "dim3")]
    pub fn new(axis: impl AsPrecise<Out = Vect>) -> Self {
        Self(RevoluteJoint::new(axis.as_precise()))
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

    /// Set the spring-like model used by the motor to reach the desired target velocity and position.
    #[must_use]
    pub fn motor_model(mut self, model: MotorModel) -> Self {
        self.0.set_motor_model(model);
        self
    }

    /// Sets the target velocity this motor needs to reach.
    #[must_use]
    pub fn motor_velocity(mut self, target_vel: Real, factor: Real) -> Self {
        self.0.set_motor_velocity(target_vel, factor);
        self
    }

    /// Sets the target angle this motor needs to reach.
    #[must_use]
    pub fn motor_position(mut self, target_pos: Real, stiffness: Real, damping: Real) -> Self {
        self.0.set_motor_position(target_pos, stiffness, damping);
        self
    }

    /// Configure both the target angle and target velocity of the motor.
    #[must_use]
    pub fn motor(
        mut self,
        target_pos: Real,
        target_vel: Real,
        stiffness: Real,
        damping: Real,
    ) -> Self {
        self.0.set_motor(target_pos, target_vel, stiffness, damping);
        self
    }

    /// Sets the maximum force the motor can deliver.
    #[must_use]
    pub fn motor_max_force(mut self, max_force: Real) -> Self {
        self.0.set_motor_max_force(max_force);
        self
    }

    /// Sets the `[min,max]` limit angles attached bodies can rotate along the joint’s principal axis.
    #[must_use]
    pub fn limits(mut self, limits: [Real; 2]) -> Self {
        self.0.set_limits(limits);
        self
    }

    /// Builds the revolute joint.
    #[must_use]
    pub fn build(self) -> RevoluteJoint {
        self.0
    }
}

impl From<RevoluteJointBuilder> for GenericJoint {
    fn from(joint: RevoluteJointBuilder) -> GenericJoint {
        joint.0.into()
    }
}
