use bevy::prelude::*;
use rapier::dynamics::{JointHandle, JointParams, RigidBodyHandle};
use rapier::geometry::ColliderHandle;
use rapier::math::{Isometry, Translation, Vector};
#[cfg(feature = "dim2")]
use rapier::na::{Complex, UnitComplex};
#[cfg(feature = "dim3")]
use rapier::na::{Quaternion, UnitQuaternion};

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

/// A component representing a joint added to the JointSet resource.
///
/// This component should not be created manually. It is automatically created and
/// added to an entity by the `JointBuilderComponent`.
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

    /// The Rapier handle of the joint.
    pub fn handle(&self) -> JointHandle {
        self.handle
    }

    /// The first Bevy entity affected by this joint.
    pub fn entity1(&self) -> Entity {
        self.entity1
    }

    /// The second Bevy entity affected by this joint.
    pub fn entity2(&self) -> Entity {
        self.entity2
    }
}

/// Component responsible for initializing a Rapier joint.
///
/// This is a transient component that will be automatically replaced by a `JointHandleComponent`
/// once the Rapier joint it describes has been created and added to the `JointSet` resource.
pub struct JointBuilderComponent {
    pub(crate) params: JointParams,
    pub(crate) entity1: Entity,
    pub(crate) entity2: Entity,
}

impl JointBuilderComponent {
    /// Initializes a joint builder from the given joint params and the entities attached to this joint.
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

/// A component to store the previous position of a body to use for
/// interpolation between steps
pub struct PhysicsInterpolationComponent(pub Isometry<f32>);

impl PhysicsInterpolationComponent {
    /// Create a new PhysicsInterpolationComponent from a translation and rotation
    #[cfg(feature = "dim2")]
    pub fn new(translation: Vec2, rotation: f32) -> Self {
        let (s, c) = rotation.sin_cos();
        Self(Isometry::from_parts(
            Translation::from(Vector::new(translation.x(), translation.y())),
            UnitComplex::from_complex(Complex::new(c, s)),
        ))
    }

    /// Create a new PhysicsInterpolationComponent from a translation and rotation
    #[cfg(feature = "dim3")]
    pub fn new(translation: Vec3, rotation: Quat) -> Self {
        Self(Isometry::from_parts(
            Translation::from(Vector::new(
                translation.x(),
                translation.y(),
                translation.z(),
            )),
            UnitQuaternion::from_quaternion(Quaternion::new(
                rotation.x(),
                rotation.y(),
                rotation.z(),
                rotation.w(),
            )),
        ))
    }
}
