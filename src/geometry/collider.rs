#[cfg(feature = "dim3")]
use crate::geometry::VHACDParameters;
use bevy::prelude::*;
use bevy::reflect::FromReflect;
#[cfg(feature = "dim3")]
use bevy::utils::HashMap;
use bevy::utils::HashSet;
use rapier::prelude::{ColliderHandle, InteractionGroups, SharedShape};

use crate::dynamics::{CoefficientCombineRule, MassProperties};
use crate::math::Vect;

/// The Rapier handle of a collider that was inserted to the physics scene.
#[derive(Copy, Clone, Debug, Component)]
pub struct RapierColliderHandle(pub ColliderHandle);

/// A component which will be replaced by the specified collider type after the referenced mesh become available.
#[cfg(feature = "dim3")]
#[derive(Component, Debug, Clone)]
pub struct AsyncCollider {
    /// Mesh handle to use for collider generation.
    pub handle: Handle<Mesh>,
    /// Collider type that will be generated.
    pub shape: ComputedColliderShape,
}

/// A component which will be replaced the specified collider types on children with meshes after the referenced scene become available.
#[cfg(feature = "dim3")]
#[derive(Component, Debug, Clone)]
pub struct AsyncSceneCollider {
    /// Scene handle to use for colliders generation.
    pub handle: Handle<Scene>,
    /// Collider type for each scene mesh not included in [`named_shapes`]. If [`None`], then all
    /// shapes will be skipped for processing except [`named_shapes`].
    pub shape: Option<ComputedColliderShape>,
    /// Shape types for meshes by name. If shape is [`None`], then it will be skipped for
    /// processing.
    pub named_shapes: HashMap<String, Option<ComputedColliderShape>>,
}

/// Shape type based on a Bevy mesh asset.
#[cfg(feature = "dim3")]
#[derive(Debug, Clone)]
pub enum ComputedColliderShape {
    /// Triangle-mesh.
    TriMesh,
    /// Convex decomposition.
    ConvexDecomposition(VHACDParameters),
}

/// A geometric entity that can be attached to a body so it can be affected by contacts
/// and intersection queries.
#[derive(Component, Clone)] // TODO: Reflect
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

/// Overwrites the default application of [`GlobalTransform::scale`] to collider shapes.
#[derive(Copy, Clone, Debug, PartialEq, Component, Reflect, FromReflect)]
pub enum ColliderScale {
    /// This scale will be multiplied with the scale in the [`GlobalTransform`] component
    /// before being applied to the collider.
    Relative(Vect),
    /// This scale will replace the one specified in the [`GlobalTransform`] component.
    Absolute(Vect),
}

/// Indicates whether or not the collider is a sensor.
#[derive(Copy, Clone, Default, Debug, PartialEq, Component, Reflect, FromReflect)]
#[reflect(Component, PartialEq)]
pub struct Sensor;

/// Custom mass-properties of a collider.
#[derive(Copy, Clone, Debug, PartialEq, Component, Reflect, FromReflect)]
#[reflect(Component, PartialEq)]
pub enum ColliderMassProperties {
    /// The mass-properties are computed automatically from the collider’s shape and this density.
    Density(f32),
    /// The mass-properties of the collider are replaced by the ones specified here.
    MassProperties(MassProperties),
}

impl Default for ColliderMassProperties {
    fn default() -> Self {
        Self::Density(1.0)
    }
}

/// The friction affecting a collider.
#[derive(Copy, Clone, Debug, PartialEq, Component, Reflect, FromReflect)]
#[reflect(Component, PartialEq)]
pub struct Friction {
    /// The friction coefficient of a collider.
    ///
    /// The greater the value, the stronger the friction forces will be.
    /// Should be `>= 0`.
    pub coefficient: f32,
    /// The rule applied to combine the friction coefficients of two colliders in contact.
    pub combine_rule: CoefficientCombineRule,
}

impl Default for Friction {
    fn default() -> Self {
        Self {
            coefficient: 0.5,
            combine_rule: CoefficientCombineRule::Average,
        }
    }
}

impl Friction {
    /// Creates a `Friction` component from the given friction coefficient, and using the default
    /// `CoefficientCombineRule::Average` coefficient combine rule.
    pub const fn new(coefficient: f32) -> Self {
        Self {
            coefficient,
            combine_rule: CoefficientCombineRule::Average,
        }
    }

    /// Creates a `Friction` component from the given friction coefficient, and using the default
    /// `CoefficientCombineRule::Average` coefficient combine rule.
    pub const fn coefficient(coefficient: f32) -> Self {
        Self {
            coefficient,
            combine_rule: CoefficientCombineRule::Average,
        }
    }
}

/// The restitution affecting a collider.
#[derive(Copy, Clone, Debug, PartialEq, Component, Reflect, FromReflect)]
#[reflect(Component, PartialEq)]
pub struct Restitution {
    /// The restitution coefficient of a collider.
    ///
    /// The greater the value, the stronger the restitution forces will be.
    /// Should be `>= 0`.
    pub coefficient: f32,
    /// The rule applied to combine the friction coefficients of two colliders in contact.
    pub combine_rule: CoefficientCombineRule,
}

impl Restitution {
    /// Creates a `Restitution` component from the given restitution coefficient, and using the default
    /// `CoefficientCombineRule::Average` coefficient combine rule.
    pub const fn new(coefficient: f32) -> Self {
        Self {
            coefficient,
            combine_rule: CoefficientCombineRule::Average,
        }
    }

    /// Creates a `Restitution` component from the given restitution coefficient, and using the default
    /// `CoefficientCombineRule::Average` coefficient combine rule.
    pub const fn coefficient(coefficient: f32) -> Self {
        Self {
            coefficient,
            combine_rule: CoefficientCombineRule::Average,
        }
    }
}

impl Default for Restitution {
    fn default() -> Self {
        Self {
            coefficient: 0.0,
            combine_rule: CoefficientCombineRule::Average,
        }
    }
}

bitflags::bitflags! {
    #[derive(Component, Reflect, FromReflect)]
    #[reflect(Component, Hash, PartialEq)]
    #[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
    /// Flags affecting whether or not collision-detection happens between two colliders
    /// depending on the type of rigid-bodies they are attached to.
    pub struct ActiveCollisionTypes: u16 {
        /// Enable collision-detection between a collider attached to a dynamic body
        /// and another collider attached to a dynamic body.
        const DYNAMIC_DYNAMIC = 0b0000_0000_0000_0001;
        /// Enable collision-detection between a collider attached to a dynamic body
        /// and another collider attached to a kinematic body.
        const DYNAMIC_KINEMATIC = 0b0000_0000_0000_1100;
        /// Enable collision-detection between a collider attached to a dynamic body
        /// and another collider attached to a fixed body (or not attached to any body).
        const DYNAMIC_STATIC  = 0b0000_0000_0000_0010;
        /// Enable collision-detection between a collider attached to a kinematic body
        /// and another collider attached to a kinematic body.
        const KINEMATIC_KINEMATIC = 0b1100_1100_0000_0000;

        /// Enable collision-detection between a collider attached to a kinematic body
        /// and another collider attached to a fixed body (or not attached to any body).
        const KINEMATIC_STATIC = 0b0010_0010_0000_0000;

        /// Enable collision-detection between a collider attached to a fixed body (or
        /// not attached to any body) and another collider attached to a fixed body (or
        /// not attached to any body).
        const STATIC_STATIC = 0b0000_0000_0010_0000;
    }
}

impl Default for ActiveCollisionTypes {
    fn default() -> Self {
        Self::DYNAMIC_DYNAMIC | Self::DYNAMIC_KINEMATIC | Self::DYNAMIC_STATIC
    }
}

impl From<ActiveCollisionTypes> for rapier::geometry::ActiveCollisionTypes {
    fn from(collision_types: ActiveCollisionTypes) -> rapier::geometry::ActiveCollisionTypes {
        rapier::geometry::ActiveCollisionTypes::from_bits(collision_types.bits)
            .expect("Internal error: invalid active events conversion.")
    }
}

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
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, Component, Reflect, FromReflect)]
#[reflect(Component, Hash, PartialEq)]
pub struct CollisionGroups {
    /// Groups memberships.
    pub memberships: u32,
    /// Groups filter.
    pub filters: u32,
}

impl Default for CollisionGroups {
    fn default() -> Self {
        Self {
            memberships: u32::MAX,
            filters: u32::MAX,
        }
    }
}

impl CollisionGroups {
    /// Creates a new collision-groups with the given membership masks and filter masks.
    pub const fn new(memberships: u32, filters: u32) -> Self {
        Self {
            memberships,
            filters,
        }
    }
}

impl From<CollisionGroups> for InteractionGroups {
    fn from(collision_groups: CollisionGroups) -> InteractionGroups {
        InteractionGroups {
            memberships: collision_groups.memberships,
            filter: collision_groups.filters,
        }
    }
}

/// Pairwise constraints resolution filtering using bit masks.
///
/// This follows the same rules as the `CollisionGroups`.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, Component, Reflect, FromReflect)]
#[reflect(Component, Hash, PartialEq)]
pub struct SolverGroups {
    /// Groups memberships.
    pub memberships: u32,
    /// Groups filter.
    pub filters: u32,
}

impl Default for SolverGroups {
    fn default() -> Self {
        Self {
            memberships: u32::MAX,
            filters: u32::MAX,
        }
    }
}

impl SolverGroups {
    /// Creates a new collision-groups with the given membership masks and filter masks.
    pub const fn new(memberships: u32, filters: u32) -> Self {
        Self {
            memberships,
            filters,
        }
    }
}

impl From<SolverGroups> for InteractionGroups {
    fn from(solver_groups: SolverGroups) -> InteractionGroups {
        InteractionGroups {
            memberships: solver_groups.memberships,
            filter: solver_groups.filters,
        }
    }
}

bitflags::bitflags! {
    #[derive(Default, Component, Reflect, FromReflect)]
    #[reflect(Component)]
    #[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
    /// Flags affecting the behavior of the constraints solver for a given contact manifold.
    pub struct ActiveHooks: u32 {
        /// If set, Rapier will call `PhysicsHooks::filter_contact_pair` whenever relevant.
        const FILTER_CONTACT_PAIRS = 0b0001;
        /// If set, Rapier will call `PhysicsHooks::filter_intersection_pair` whenever relevant.
        const FILTER_INTERSECTION_PAIR = 0b0010;
        /// If set, Rapier will call `PhysicsHooks::modify_solver_contact` whenever relevant.
        const MODIFY_SOLVER_CONTACTS = 0b0100;
    }
}

impl From<ActiveHooks> for rapier::pipeline::ActiveHooks {
    fn from(active_hooks: ActiveHooks) -> rapier::pipeline::ActiveHooks {
        rapier::pipeline::ActiveHooks::from_bits(active_hooks.bits)
            .expect("Internal error: invalid active events conversion.")
    }
}

bitflags::bitflags! {
    #[derive(Default, Component, Reflect, FromReflect)]
    #[reflect(Component)]
    #[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
    /// Flags affecting the events generated for this collider.
    pub struct ActiveEvents: u32 {
        /// If set, Rapier will call `EventHandler::handle_intersection_event` and
        /// `EventHandler::handle_contact_event` whenever relevant for this collider.
        const COLLISION_EVENTS = 0b0001;
    }
}

impl From<ActiveEvents> for rapier::pipeline::ActiveEvents {
    fn from(active_events: ActiveEvents) -> rapier::pipeline::ActiveEvents {
        rapier::pipeline::ActiveEvents::from_bits(active_events.bits)
            .expect("Internal error: invalid active events conversion.")
    }
}

/// Component which will be filled (if present) with a list of entities with which the current entity is currently in contact.
#[derive(Component, Default, Reflect, FromReflect)]
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
