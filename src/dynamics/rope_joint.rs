use crate::dynamics::{GenericJoint, GenericJointBuilder};
use crate::math::{AsPrecise, Real, Vect};
use rapier::dynamics::{JointAxesMask, JointAxis, JointLimits, JointMotor, MotorModel};

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Copy, Clone, Debug, PartialEq)]
#[repr(transparent)]
/// A rope joint, limits the maximum distance between two bodies
pub struct RopeJoint {
    data: GenericJoint,
}

impl RopeJoint {
    /// Creates a new rope joint limiting the max distance between to bodies
    pub fn new() -> Self {
        let data = GenericJointBuilder::new(JointAxesMask::FREE_FIXED_AXES)
            .coupled_axes(JointAxesMask::LIN_AXES)
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

    /// The principal axis of the joint, expressed in the local-space of the first rigid-body.
    #[must_use]
    pub fn local_axis1(&self) -> Vect {
        self.data.local_axis1()
    }

    /// Sets the principal axis of the joint, expressed in the local-space of the first rigid-body.
    pub fn set_local_axis1(&mut self, axis1: impl AsPrecise<Out = Vect>) -> &mut Self {
        self.data.set_local_axis1(axis1.as_precise());
        self
    }

    /// The principal axis of the joint, expressed in the local-space of the second rigid-body.
    #[must_use]
    pub fn local_axis2(&self) -> Vect {
        self.data.local_axis2()
    }

    /// Sets the principal axis of the joint, expressed in the local-space of the second rigid-body.
    pub fn set_local_axis2(&mut self, axis2: impl AsPrecise<Out = Vect>) -> &mut Self {
        self.data.set_local_axis2(axis2.as_precise());
        self
    }

    /// The motor affecting the joint’s translational degree of freedom.
    #[must_use]
    pub fn motor(&self, axis: JointAxis) -> Option<&JointMotor> {
        self.data.motor(axis)
    }

    /// Set the spring-like model used by the motor to reach the desired target velocity and position.
    pub fn set_motor_model(&mut self, model: MotorModel) -> &mut Self {
        self.data.set_motor_model(JointAxis::X, model);
        self.data.set_motor_model(JointAxis::Y, model);
        #[cfg(feature = "dim3")]
        self.data.set_motor_model(JointAxis::Z, model);
        self
    }

    /// Sets the target velocity this motor needs to reach.
    pub fn set_motor_velocity(&mut self, target_vel: Real, factor: Real) -> &mut Self {
        self.data
            .set_motor_velocity(JointAxis::X, target_vel, factor);
        self.data
            .set_motor_velocity(JointAxis::Y, target_vel, factor);
        #[cfg(feature = "dim3")]
        self.data
            .set_motor_velocity(JointAxis::Z, target_vel, factor);
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
            .set_motor_position(JointAxis::X, target_pos, stiffness, damping);
        self.data
            .set_motor_position(JointAxis::Y, target_pos, stiffness, damping);
        #[cfg(feature = "dim3")]
        self.data
            .set_motor_position(JointAxis::Z, target_pos, stiffness, damping);
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
            .set_motor(JointAxis::X, target_pos, target_vel, stiffness, damping);
        self.data
            .set_motor(JointAxis::Y, target_pos, target_vel, stiffness, damping);
        #[cfg(feature = "dim3")]
        self.data
            .set_motor(JointAxis::Y, target_pos, target_vel, stiffness, damping);
        self
    }

    /// Sets the maximum force the motor can deliver.
    pub fn set_motor_max_force(&mut self, max_force: Real) -> &mut Self {
        self.data.set_motor_max_force(JointAxis::X, max_force);
        self.data.set_motor_max_force(JointAxis::Y, max_force);
        #[cfg(feature = "dim3")]
        self.data.set_motor_max_force(JointAxis::Z, max_force);
        self
    }

    /// The limit distance attached bodies can translate along the joint’s principal axis.
    #[must_use]
    pub fn limits(&self, axis: JointAxis) -> Option<&JointLimits<Real>> {
        self.data.limits(axis)
    }

    /// Sets the `[min,max]` limit distances attached bodies can translate.
    pub fn set_limits(&mut self, limits: [Real; 2]) -> &mut Self {
        self.data.set_limits(JointAxis::X, limits);
        self.data.set_limits(JointAxis::Y, limits);
        #[cfg(feature = "dim3")]
        self.data.set_limits(JointAxis::Z, limits);
        self
    }
}

impl Default for RopeJoint {
    fn default() -> Self {
        Self::new()
    }
}

impl From<RopeJoint> for GenericJoint {
    fn from(joint: RopeJoint) -> GenericJoint {
        joint.data
    }
}

/// Create rope joints using the builder pattern.
///
/// A rope joint, limits the maximum distance between two bodies.
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct RopeJointBuilder(RopeJoint);

impl RopeJointBuilder {
    /// Creates a new builder for rope joints.
    ///
    /// This axis is expressed in the local-space of both rigid-bodies.
    pub fn new() -> Self {
        Self(RopeJoint::new())
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

    /// Sets the principal axis of the joint, expressed in the local-space of the first rigid-body.
    #[must_use]
    pub fn local_axis1(mut self, axis1: Vect) -> Self {
        self.0.set_local_axis1(axis1);
        self
    }

    /// Sets the principal axis of the joint, expressed in the local-space of the second rigid-body.
    #[must_use]
    pub fn local_axis2(mut self, axis2: Vect) -> Self {
        self.0.set_local_axis2(axis2);
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
    pub fn set_motor(
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

    /// Sets the `[min,max]` limit distances attached bodies can translate.
    #[must_use]
    pub fn limits(mut self, limits: [Real; 2]) -> Self {
        self.0.set_limits(limits);
        self
    }

    /// Builds the rope joint.
    #[must_use]
    pub fn build(self) -> RopeJoint {
        self.0
    }
}

impl Default for RopeJointBuilder {
    fn default() -> Self {
        Self::new()
    }
}

impl From<RopeJointBuilder> for GenericJoint {
    fn from(joint: RopeJointBuilder) -> GenericJoint {
        joint.0.into()
    }
}
