use std::fmt;
use std::ops::{Deref, DerefMut};

#[cfg(all(feature = "dim3", feature = "async-collider"))]
use {crate::geometry::VHACDParameters, bevy::utils::HashMap};

use bevy::prelude::*;

use bevy::utils::HashSet;
use rapier::geometry::Shape;
use rapier::prelude::{InteractionGroups, SharedShape};

use crate::math::Vect;

pub use rapier::prelude::{
    ActiveCollisionTypes, ActiveEvents, ActiveHooks, ColliderHandle, ColliderMassProperties,
    Friction, Group, Restitution,
};

/// A component which will be replaced by the specified collider type after the referenced mesh become available.
#[cfg(all(feature = "dim3", feature = "async-collider"))]
#[derive(Component, Debug, Clone, Default)]
pub struct AsyncCollider(pub ComputedColliderShape);

/// A component which will be replaced the specified collider types on children with meshes after the referenced scene become available.
#[cfg(all(feature = "dim3", feature = "async-collider"))]
#[derive(Component, Debug, Clone)]
pub struct AsyncSceneCollider {
    /// Collider type for each scene mesh not included in [`named_shapes`]. If [`None`], then all
    /// shapes will be skipped for processing except [`named_shapes`].
    pub shape: Option<ComputedColliderShape>,
    /// Shape types for meshes by name. If shape is [`None`], then it will be skipped for
    /// processing.
    pub named_shapes: HashMap<String, Option<ComputedColliderShape>>,
}

#[cfg(all(feature = "dim3", feature = "async-collider"))]
impl Default for AsyncSceneCollider {
    fn default() -> Self {
        Self {
            shape: Some(ComputedColliderShape::TriMesh),
            named_shapes: Default::default(),
        }
    }
}

/// Shape type based on a Bevy mesh asset.
#[cfg(all(feature = "dim3", feature = "async-collider"))]
#[derive(Debug, Clone, Default)]
pub enum ComputedColliderShape {
    /// Triangle-mesh.
    #[default]
    TriMesh,
    /// Convex hull.
    ConvexHull,
    /// Convex decomposition.
    ConvexDecomposition(VHACDParameters),
}

/// A geometric entity that can be attached to a [`RigidBody`] so it can be affected by contacts
/// and intersection queries.
///
/// Related components:
/// - [`ColliderMassProperties`]
/// - [`Friction`]
/// - [`Restitution`]
/// - [`Sensor`]
/// - [`CollisionGroups`]
/// - [`SolverGroups`]
/// - [`ActiveCollisionTypes`]
/// - [`ActiveEvents`]
/// - [`ContactForceEventThreshold`]
/// - [`CollidingEntities`]
/// - [`ColliderScale`]
/// - [`ColliderDisabled`]
#[derive(Component, Clone)] // TODO: Reflect
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct Collider {
    /// The raw shape from Rapier.
    pub raw: SharedShape,
    pub(crate) unscaled: SharedShape,
    pub(crate) scale: Vect,
}

impl From<SharedShape> for Collider {
    fn from(shared_shape: SharedShape) -> Collider {
        Collider {
            raw: shared_shape.clone(),
            unscaled: shared_shape,
            scale: Vect::ONE,
        }
    }
}

impl<'a> From<&'a Collider> for &'a dyn Shape {
    fn from(collider: &'a Collider) -> &'a dyn Shape {
        &*collider.raw
    }
}

impl fmt::Debug for Collider {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        todo!() // Merge https://github.com/dimforge/parry/pull/179 first.
                // self.as_typed_shape().fmt(f)
    }
}

/// Overwrites the default application of [`GlobalTransform::scale`] to a [`Collider`]'s shapes.
#[derive(Copy, Clone, Debug, PartialEq, Component, Reflect)]
pub enum ColliderScale {
    /// This scale will be multiplied with the scale in the [`GlobalTransform`] component
    /// before being applied to the collider.
    Relative(Vect),
    /// This scale will replace the one specified in the [`GlobalTransform`] component.
    Absolute(Vect),
}

/// Indicates whether or not the [`Collider`] is a sensor.
#[derive(Copy, Clone, Default, Debug, PartialEq, Eq, Component, Reflect)]
#[reflect(Component, PartialEq)]
pub struct Sensor;

/// Pairwise collision filtering using bit masks.
///
/// This filtering method is based on two 32-bit values:
/// - The interaction groups memberships.
/// - The interaction groups filter.
///
/// An interaction is allowed between two filters `a` and `b` when two conditions
/// are met simultaneously:
/// - The groups membership of `a` has at least one bit set to `1` in common with the groups filter of `b`.
/// - The groups membership of `b` has at least one bit set to `1` in common with the groups filter of `a`.
///
/// In other words, interactions are allowed between two filter iff. the following condition is met:
/// ```ignore
/// (self.memberships & rhs.filter) != 0 && (rhs.memberships & self.filter) != 0
/// ```
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Hash, Component, Reflect)]
#[reflect(Component, Hash, PartialEq)]
pub struct CollisionGroups(pub InteractionGroups);

impl CollisionGroups {
    /// Creates a new collision-groups with the given membership masks and filter masks.
    pub const fn new(memberships: Group, filter: Group) -> Self {
        Self(InteractionGroups {
            memberships,
            filter,
        })
    }
}

impl Deref for CollisionGroups {
    type Target = InteractionGroups;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for CollisionGroups {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

/// Pairwise constraints resolution filtering using bit masks.
///
/// This follows the same rules as the [`CollisionGroups`].
#[derive(Copy, Clone, Default, Debug, PartialEq, Eq, Hash, Component, Reflect)]
#[reflect(Component, Hash, PartialEq)]
pub struct SolverGroups(pub InteractionGroups);

impl SolverGroups {
    /// Creates a new solver-groups with the given membership masks and filter masks.
    pub const fn new(memberships: Group, filter: Group) -> Self {
        Self(InteractionGroups {
            memberships,
            filter,
        })
    }
}

impl Deref for SolverGroups {
    type Target = InteractionGroups;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for SolverGroups {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

/// The total force magnitude beyond which a [`ContactForceEvent`] can be emitted.
///
/// This requires that the [`ActiveEvents::CONTACT_FORCE_EVENTS`] flag is set on the
/// entity.
#[derive(Copy, Clone, PartialEq, Component, Reflect)]
#[reflect(Component)]
pub struct ContactForceEventThreshold(pub f32);

impl Default for ContactForceEventThreshold {
    fn default() -> Self {
        Self(f32::MAX)
    }
}

/// Component which will be filled (if present) with a list of entities with which the current
/// entity is currently in contact.
///
/// This currently only updates when on an entity with a `Collider`, and if the
/// [`ActiveEvents::COLLISION_EVENTS`] is set on this entity or the entity it
/// collided with.
#[derive(Component, Default, Reflect)]
#[reflect(Component)]
pub struct CollidingEntities(pub(crate) HashSet<Entity>);

impl CollidingEntities {
    /// Returns the number of colliding entities.
    #[must_use]
    pub fn len(&self) -> usize {
        self.0.len()
    }

    /// Returns `true` if there is no colliding entities.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }

    /// Returns `true` if the collisions contains the specified entity.
    #[must_use]
    pub fn contains(&self, entity: Entity) -> bool {
        self.0.contains(&entity)
    }

    /// An iterator visiting all colliding entities in arbitrary order.
    pub fn iter(&self) -> impl Iterator<Item = Entity> + '_ {
        self.0.iter().copied()
    }
}

/// Indicates whether the collider is disabled explicitly by the user.
#[derive(Copy, Clone, Default, Debug, PartialEq, Eq, Component, Reflect)]
#[reflect(Component, PartialEq)]
pub struct ColliderDisabled;

/// We restrict the scaling increment to 1.0e-4, to avoid numerical jitter
/// due to the extraction of scaling factor from the GlobalTransform matrix.
pub fn get_snapped_scale(scale: Vect) -> Vect {
    fn snap_value(new: f32) -> f32 {
        const PRECISION: f32 = 1.0e4;
        (new * PRECISION).round() / PRECISION
    }

    Vect {
        x: snap_value(scale.x),
        y: snap_value(scale.y),
        #[cfg(feature = "dim3")]
        z: snap_value(scale.z),
    }
}

/// Indicates whether the collider was created in the physics backend.
#[derive(Copy, Clone, Default, Debug, PartialEq, Eq, Component, Reflect)]
#[reflect(Component, PartialEq)]
pub struct ColliderCreated;
