use crate::dynamics::{FixedJoint, PrismaticJoint, RevoluteJoint};
use crate::math::{Real, Rot, Vect};
use bevy::prelude::*;
use rapier::dynamics::{
    GenericJoint as RapierGenericJoint, ImpulseJointHandle, JointAxesMask, JointAxis, JointLimits,
    JointMotor, MotorModel, MultibodyJointHandle,
};
use rapier::math::SPATIAL_DIM;

#[cfg(feature = "dim3")]
use crate::dynamics::SphericalJoint;

#[derive(Copy, Clone, Debug, PartialEq, Default)]
#[repr(transparent)]
pub struct GenericJoint {
    pub raw: RapierGenericJoint,
}

impl GenericJoint {
    pub fn into_rapier(mut self, physics_scale: Real) -> RapierGenericJoint {
        self.raw.local_frame1.translation.vector /= physics_scale;
        self.raw.local_frame2.translation.vector /= physics_scale;

        for limit in &mut self.raw.limits {
            limit.min /= physics_scale;
            limit.max /= physics_scale;
        }

        for motor in &mut self.raw.motors {
            motor.target_vel /= physics_scale;
            motor.target_pos /= physics_scale;
        }

        self.raw
    }
}

/*
 * NOTE: the following are copy-pasted from Rapier’s GenericJoint, to match its
 *       construction methods, but using glam types.
 */

impl GenericJoint {
    /// Creates a new generic joint that locks the specified degrees of freedom.
    #[must_use]
    pub fn new(locked_axes: JointAxesMask) -> Self {
        Self {
            raw: RapierGenericJoint::new(locked_axes),
        }
    }

    /// The set of axes locked by this joint.
    pub fn locked_axes(&self) -> JointAxesMask {
        self.raw.locked_axes
    }

    /// Add the specified axes to the set of axes locked by this joint.
    pub fn lock_axes(&mut self, axes: JointAxesMask) -> &mut Self {
        self.raw.lock_axes(axes);
        self
    }

    /// The basis of this joint, expressed in the first rigid-body’s local-space.
    #[must_use]
    pub fn local_basis1(&self) -> Rot {
        #[cfg(feature = "dim2")]
        return self.raw.local_frame1.rotation.angle();
        #[cfg(feature = "dim3")]
        return self.raw.local_frame1.rotation.into();
    }

    /// Sets the joint’s frame, expressed in the first rigid-body’s local-space.
    pub fn set_local_basis1(&mut self, local_basis: Rot) -> &mut Self {
        #[cfg(feature = "dim2")]
        {
            self.raw.local_frame1.rotation = na::UnitComplex::new(local_basis);
        }
        #[cfg(feature = "dim3")]
        {
            self.raw.local_frame1.rotation = local_basis.into();
        }
        self
    }

    /// The basis of this joint, expressed in the second rigid-body’s local-space.
    #[must_use]
    pub fn local_basis2(&self) -> Rot {
        #[cfg(feature = "dim2")]
        return self.raw.local_frame2.rotation.angle();
        #[cfg(feature = "dim3")]
        return self.raw.local_frame2.rotation.into();
    }

    /// Sets the joint’s frame, expressed in the second rigid-body’s local-space.
    pub fn set_local_basis2(&mut self, local_basis: Rot) -> &mut Self {
        #[cfg(feature = "dim2")]
        {
            self.raw.local_frame2.rotation = na::UnitComplex::new(local_basis);
        }
        #[cfg(feature = "dim3")]
        {
            self.raw.local_frame2.rotation = local_basis.into();
        }
        self
    }

    /// The principal (local X) axis of this joint, expressed in the first rigid-body’s local-space.
    #[must_use]
    pub fn local_axis1(&self) -> Vect {
        (*self.raw.local_axis1()).into()
    }

    /// Sets the principal (local X) axis of this joint, expressed in the first rigid-body’s local-space.
    pub fn set_local_axis1(&mut self, local_axis: Vect) -> &mut Self {
        self.raw.set_local_axis1(local_axis.try_into().unwrap());
        self
    }

    /// The principal (local X) axis of this joint, expressed in the second rigid-body’s local-space.
    #[must_use]
    pub fn local_axis2(&self) -> Vect {
        (*self.raw.local_axis2()).into()
    }

    /// Sets the principal (local X) axis of this joint, expressed in the second rigid-body’s local-space.
    pub fn set_local_axis2(&mut self, local_axis: Vect) -> &mut Self {
        self.raw.set_local_axis2(local_axis.try_into().unwrap());
        self
    }

    /// The anchor of this joint, expressed in the first rigid-body’s local-space.
    #[must_use]
    pub fn local_anchor1(&self) -> Vect {
        self.raw.local_anchor1().into()
    }

    /// Sets anchor of this joint, expressed in the first rigid-body’s local-space.
    pub fn set_local_anchor1(&mut self, anchor1: Vect) -> &mut Self {
        self.raw.set_local_anchor1(anchor1.into());
        self
    }

    /// The anchor of this joint, expressed in the second rigid-body’s local-space.
    #[must_use]
    pub fn local_anchor2(&self) -> Vect {
        self.raw.local_anchor2().into()
    }

    /// Sets anchor of this joint, expressed in the second rigid-body’s local-space.
    pub fn set_local_anchor2(&mut self, anchor2: Vect) -> &mut Self {
        self.raw.set_local_anchor2(anchor2.into());
        self
    }

    /// The joint limits along the specified axis.
    #[must_use]
    pub fn limits(&self, axis: JointAxis) -> Option<&JointLimits<Real>> {
        self.raw.limits(axis)
    }

    /// Sets the joint limits along the specified axis.
    pub fn set_limits(&mut self, axis: JointAxis, limits: [Real; 2]) -> &mut Self {
        self.raw.set_limits(axis, limits);
        self
    }

    /// The spring-like motor model along the specified axis of this joint.
    #[must_use]
    pub fn motor_model(&self, axis: JointAxis) -> Option<MotorModel> {
        self.raw.motor_model(axis)
    }

    /// Set the spring-like model used by the motor to reach the desired target velocity and position.
    pub fn set_motor_model(&mut self, axis: JointAxis, model: MotorModel) -> &mut Self {
        self.raw.set_motor_model(axis, model);
        self
    }

    /// Sets the target velocity this motor needs to reach.
    pub fn set_motor_velocity(
        &mut self,
        axis: JointAxis,
        target_vel: Real,
        factor: Real,
    ) -> &mut Self {
        self.raw.set_motor_velocity(axis, target_vel, factor);
        self
    }

    /// Sets the target angle this motor needs to reach.
    pub fn set_motor_position(
        &mut self,
        axis: JointAxis,
        target_pos: Real,
        stiffness: Real,
        damping: Real,
    ) -> &mut Self {
        self.raw
            .set_motor_position(axis, target_pos, stiffness, damping);
        self
    }

    /// Sets the maximum force the motor can deliver along the specified axis.
    pub fn set_motor_max_force(&mut self, axis: JointAxis, max_force: Real) -> &mut Self {
        self.raw.set_motor_max_force(axis, max_force);
        self
    }

    /// The motor affecting the joint’s degree of freedom along the specified axis.
    #[must_use]
    pub fn motor(&self, axis: JointAxis) -> Option<&JointMotor> {
        self.raw.motor(axis)
    }

    /// Configure both the target angle and target velocity of the motor.
    pub fn set_motor(
        &mut self,
        axis: JointAxis,
        target_pos: Real,
        target_vel: Real,
        stiffness: Real,
        damping: Real,
    ) -> &mut Self {
        self.raw
            .set_motor(axis, target_pos, target_vel, stiffness, damping);
        self
    }
}

macro_rules! joint_conversion_methods(
    ($as_joint: ident, $as_joint_mut: ident, $Joint: ty, $axes: expr) => {
        /// Converts the joint to its specific variant, if it is one.
        #[must_use]
        pub fn $as_joint(&self) -> Option<&$Joint> {
            if self.locked_axes() == $axes {
                // SAFETY: this is OK because the target joint type is
                //         a `repr(transparent)` newtype of `Joint`.
                Some(unsafe { std::mem::transmute(self) })
            } else {
                None
            }
        }

        /// Converts the joint to its specific mutable variant, if it is one.
        #[must_use]
        pub fn $as_joint_mut(&mut self) -> Option<&mut $Joint> {
            if self.locked_axes() == $axes {
                // SAFETY: this is OK because the target joint type is
                //         a `repr(transparent)` newtype of `Joint`.
                Some(unsafe { std::mem::transmute(self) })
            } else {
                None
            }
        }
    }
);

impl GenericJoint {
    joint_conversion_methods!(
        as_revolute,
        as_revolute_mut,
        RevoluteJoint,
        JointAxesMask::LOCKED_REVOLUTE_AXES
    );
    joint_conversion_methods!(
        as_fixed,
        as_fixed_mut,
        FixedJoint,
        JointAxesMask::LOCKED_FIXED_AXES
    );
    joint_conversion_methods!(
        as_prismatic,
        as_prismatic_mut,
        PrismaticJoint,
        JointAxesMask::LOCKED_PRISMATIC_AXES
    );

    #[cfg(feature = "dim3")]
    joint_conversion_methods!(
        as_spherical,
        as_spherical_mut,
        SphericalJoint,
        JointAxesMask::LOCKED_SPHERICAL_AXES
    );
}

/// Create generic joints using the builder pattern.
#[derive(Copy, Clone, Debug)]
pub struct GenericJointBuilder(GenericJoint);

impl GenericJointBuilder {
    /// Creates a new generic joint builder.
    #[must_use]
    pub fn new(locked_axes: JointAxesMask) -> Self {
        Self(GenericJoint::new(locked_axes))
    }

    /// Sets the degrees of freedom locked by the joint.
    #[must_use]
    pub fn locked_axes(mut self, axes: JointAxesMask) -> Self {
        self.0.lock_axes(axes);
        self
    }

    /// Sets the joint’s frame, expressed in the first rigid-body’s local-space.
    #[must_use]
    pub fn local_basis1(mut self, local_basis: Rot) -> Self {
        self.0.set_local_basis1(local_basis);
        self
    }

    /// Sets the joint’s frame, expressed in the second rigid-body’s local-space.
    #[must_use]
    pub fn local_basis2(mut self, local_basis: Rot) -> Self {
        self.0.set_local_basis2(local_basis);
        self
    }

    /// Sets the principal (local X) axis of this joint, expressed in the first rigid-body’s local-space.
    #[must_use]
    pub fn local_axis1(mut self, local_axis: Vect) -> Self {
        self.0.set_local_axis1(local_axis);
        self
    }

    /// Sets the principal (local X) axis of this joint, expressed in the second rigid-body’s local-space.
    #[must_use]
    pub fn local_axis2(mut self, local_axis: Vect) -> Self {
        self.0.set_local_axis2(local_axis);
        self
    }

    /// Sets the anchor of this joint, expressed in the first rigid-body’s local-space.
    #[must_use]
    pub fn local_anchor1(mut self, anchor1: Vect) -> Self {
        self.0.set_local_anchor1(anchor1);
        self
    }

    /// Sets the anchor of this joint, expressed in the second rigid-body’s local-space.
    #[must_use]
    pub fn local_anchor2(mut self, anchor2: Vect) -> Self {
        self.0.set_local_anchor2(anchor2);
        self
    }

    /// Sets the joint limits along the specified axis.
    #[must_use]
    pub fn limits(mut self, axis: JointAxis, limits: [Real; 2]) -> Self {
        self.0.set_limits(axis, limits);
        self
    }

    // /// Sets the coupled degrees of freedom for this joint’s limits and motor.
    // #[must_use]
    // pub fn coupled_axes(mut self, axes: JointAxesMask) -> Self {
    //     self.0.coupled_axes = axes;
    //     self
    // }

    /// Set the spring-like model used by the motor to reach the desired target velocity and position.
    #[must_use]
    pub fn motor_model(mut self, axis: JointAxis, model: MotorModel) -> Self {
        self.0.set_motor_model(axis, model);
        self
    }

    /// Sets the target velocity this motor needs to reach.
    #[must_use]
    pub fn motor_velocity(mut self, axis: JointAxis, target_vel: Real, factor: Real) -> Self {
        self.0.set_motor_velocity(axis, target_vel, factor);
        self
    }

    /// Sets the target angle this motor needs to reach.
    #[must_use]
    pub fn motor_position(
        mut self,
        axis: JointAxis,
        target_pos: Real,
        stiffness: Real,
        damping: Real,
    ) -> Self {
        self.0
            .set_motor_position(axis, target_pos, stiffness, damping);
        self
    }

    /// Configure both the target angle and target velocity of the motor.
    #[must_use]
    pub fn set_motor(
        mut self,
        axis: JointAxis,
        target_pos: Real,
        target_vel: Real,
        stiffness: Real,
        damping: Real,
    ) -> Self {
        self.0
            .set_motor(axis, target_pos, target_vel, stiffness, damping);
        self
    }

    /// Sets the maximum force the motor can deliver along the specified axis.
    #[must_use]
    pub fn motor_max_force(mut self, axis: JointAxis, max_force: Real) -> Self {
        self.0.set_motor_max_force(axis, max_force);
        self
    }

    /// Builds the generic joint.
    #[must_use]
    pub fn build(self) -> GenericJoint {
        self.0
    }
}

impl Into<GenericJoint> for GenericJointBuilder {
    fn into(self) -> GenericJoint {
        self.0
    }
}
