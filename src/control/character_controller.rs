use crate::geometry::{Collider, CollisionGroups, ShapeCastHit};
use crate::math::{Real, Rot, Vect};
use bevy::prelude::*;

use crate::plugin::{RapierContext, RapierContextColliders};
pub use rapier::control::CharacterAutostep;
pub use rapier::control::CharacterLength;
use rapier::prelude::{ColliderSet, QueryFilterFlags};

/// A collision between the character and its environment during its movement.
#[derive(Copy, Clone, PartialEq, Debug)]
pub struct CharacterCollision {
    /// The entity hit by the character.
    pub entity: Entity,
    /// The position of the character when the collider was hit.
    pub character_translation: Vect,
    /// The rotation of the character when the collider was hit.
    pub character_rotation: Rot,
    /// The translation that was already applied to the character when the hit happens.
    pub translation_applied: Vect,
    /// The translations that was still waiting to be applied to the character when the hit happens.
    pub translation_remaining: Vect,
    /// Geometric information about the hit.
    pub hit: ShapeCastHit,
}

impl CharacterCollision {
    pub(crate) fn from_raw(
        ctxt: &RapierContextColliders,
        c: &rapier::control::CharacterCollision,
    ) -> Option<Self> {
        Self::from_raw_with_set(&ctxt.colliders, c, true)
    }

    pub(crate) fn from_raw_with_set(
        colliders: &ColliderSet,
        c: &rapier::control::CharacterCollision,
        details_always_computed: bool,
    ) -> Option<Self> {
        RapierContextColliders::collider_entity_with_set(colliders, c.handle).map(|entity| {
            CharacterCollision {
                entity,
                character_translation: c.character_pos.translation.vector.into(),
                #[cfg(feature = "dim2")]
                character_rotation: c.character_pos.rotation.angle(),
                #[cfg(feature = "dim3")]
                character_rotation: c.character_pos.rotation.into(),
                translation_applied: c.translation_applied.into(),
                translation_remaining: c.translation_remaining.into(),
                hit: ShapeCastHit::from_rapier(c.hit, details_always_computed),
            }
        })
    }
}

/// Options for moving a shape using `RapierContext::move_shape`.
#[derive(Clone, Debug, Copy, PartialEq)]
pub struct MoveShapeOptions {
    /// The direction that goes "up". Used to determine where the floor is, and the floor’s angle.
    pub up: Vect,
    /// A small gap to preserve between the character and its surroundings.
    ///
    /// This value should not be too large to avoid visual artifacts, but shouldn’t be too small
    /// (must not be zero) to improve numerical stability of the character controller.
    pub offset: CharacterLength,
    /// Should the character try to slide against the floor if it hits it?
    pub slide: bool,
    /// Should the character automatically step over small obstacles?
    pub autostep: Option<CharacterAutostep>,
    /// The maximum angle (radians) between the floor’s normal and the `up` vector that the
    /// character is able to climb.
    pub max_slope_climb_angle: Real,
    /// The minimum angle (radians) between the floor’s normal and the `up` vector before the
    /// character starts to slide down automatically.
    pub min_slope_slide_angle: Real,
    /// Should the character apply forces to dynamic bodies in its path?
    pub apply_impulse_to_dynamic_bodies: bool,
    /// Should the character be automatically snapped to the ground if the distance between
    /// the ground and its feet are smaller than the specified threshold?
    pub snap_to_ground: Option<CharacterLength>,
    /// Increase this number if your character appears to get stuck when sliding against surfaces.
    ///
    /// This is a small distance applied to the movement toward the contact normals of shapes hit
    /// by the character controller. This helps shape-casting not getting stuck in an always-penetrating
    /// state during the sliding calculation.
    ///
    /// This value should remain fairly small since it can introduce artificial "bumps" when sliding
    /// along a flat surface.
    pub normal_nudge_factor: Real,
}

impl Default for MoveShapeOptions {
    fn default() -> Self {
        let def = rapier::control::KinematicCharacterController::default();
        Self {
            up: def.up.into(),
            offset: def.offset,
            slide: def.slide,
            autostep: def.autostep,
            max_slope_climb_angle: def.max_slope_climb_angle,
            min_slope_slide_angle: def.min_slope_slide_angle,
            apply_impulse_to_dynamic_bodies: true,
            snap_to_ground: def.snap_to_ground,
            normal_nudge_factor: def.normal_nudge_factor,
        }
    }
}

/// A character controller for kinematic bodies and free-standing colliders.
#[derive(Clone, Debug, Component)] // TODO: Reflect
pub struct KinematicCharacterController {
    /// The translations we desire the character to move by if it doesn’t meet any obstacle.
    pub translation: Option<Vect>,
    /// The shape, and its position, to be used instead of the shape of the collider attached to
    /// the same entity is this `KinematicCharacterController`.
    pub custom_shape: Option<(Collider, Vect, Rot)>,
    /// The mass to be used for impulse of dynamic bodies. This replaces the mass of the rigid-body
    /// potentially associated to the collider attached to the same entity as this
    /// `KinematicCharacterController`.
    ///
    /// This field isn’t used if `Self::apply_impulse_to_dynamic_bodies` is set to `false`.
    pub custom_mass: Option<Real>,
    /// The direction that goes "up". Used to determine where the floor is, and the floor’s angle.
    pub up: Vect,
    /// A small gap to preserve between the character and its surroundings.
    ///
    /// This value should not be too large to avoid visual artifacts, but shouldn’t be too small
    /// (must not be zero) to improve numerical stability of the character controller.
    pub offset: CharacterLength,
    /// Should the character try to slide against the floor if it hits it?
    pub slide: bool,
    /// Should the character automatically step over small obstacles?
    pub autostep: Option<CharacterAutostep>,
    /// The maximum angle (radians) between the floor’s normal and the `up` vector that the
    /// character is able to climb.
    pub max_slope_climb_angle: Real,
    /// The minimum angle (radians) between the floor’s normal and the `up` vector before the
    /// character starts to slide down automatically.
    pub min_slope_slide_angle: Real,
    /// Should the character apply forces to dynamic bodies in its path?
    pub apply_impulse_to_dynamic_bodies: bool,
    /// Should the character be automatically snapped to the ground if the distance between
    /// the ground and its feet are smaller than the specified threshold?
    pub snap_to_ground: Option<CharacterLength>,
    /// Flags for filtering-out some categories of entities from the environment seen by the
    /// character controller.
    pub filter_flags: QueryFilterFlags,
    /// Groups for filtering-out some colliders from the environment seen by the character
    /// controller.
    pub filter_groups: Option<CollisionGroups>,
    /// Increase this number if your character appears to get stuck when sliding against surfaces.
    ///
    /// This is a small distance applied to the movement toward the contact normals of shapes hit
    /// by the character controller. This helps shape-casting not getting stuck in an always-penetrating
    /// state during the sliding calculation.
    ///
    /// This value should remain fairly small since it can introduce artificial "bumps" when sliding
    /// along a flat surface.
    pub normal_nudge_factor: Real,
}

impl KinematicCharacterController {
    pub(crate) fn to_raw(&self) -> Option<rapier::control::KinematicCharacterController> {
        let autostep = self.autostep.map(|autostep| CharacterAutostep {
            max_height: autostep.max_height,
            min_width: autostep.min_width,
            include_dynamic_bodies: autostep.include_dynamic_bodies,
        });

        Some(rapier::control::KinematicCharacterController {
            up: self.up.try_into().ok()?,
            offset: self.offset,
            slide: self.slide,
            autostep,
            max_slope_climb_angle: self.max_slope_climb_angle,
            min_slope_slide_angle: self.min_slope_slide_angle,
            snap_to_ground: self.snap_to_ground,
            normal_nudge_factor: self.normal_nudge_factor,
        })
    }
}

impl Default for KinematicCharacterController {
    fn default() -> Self {
        let def = rapier::control::KinematicCharacterController::default();
        Self {
            translation: None,
            custom_shape: None,
            custom_mass: None,
            up: def.up.into(),
            offset: def.offset,
            slide: def.slide,
            autostep: def.autostep,
            max_slope_climb_angle: def.max_slope_climb_angle,
            min_slope_slide_angle: def.min_slope_slide_angle,
            apply_impulse_to_dynamic_bodies: true,
            snap_to_ground: def.snap_to_ground,
            filter_flags: QueryFilterFlags::default() | QueryFilterFlags::EXCLUDE_SENSORS,
            filter_groups: None,
            normal_nudge_factor: def.normal_nudge_factor,
        }
    }
}

/// The output of a character control.
///
/// This component is automatically added after the first execution of a character control
/// based on the `KinematicCharacterController` component with its
/// `KinematicCharacterController::translation` set to a value other than `None`.
#[derive(Clone, PartialEq, Debug, Default, Component)]
pub struct KinematicCharacterControllerOutput {
    /// Indicates whether the shape is grounded after its kinematic movement.
    pub grounded: bool,
    /// The initial desired movement of the character if there were no obstacle.
    pub desired_translation: Vect,
    /// The translation calculated by the last character control step taking obstacles into account.
    pub effective_translation: Vect,
    /// Collisions between the character and obstacles found in its path.
    pub collisions: Vec<CharacterCollision>,
    /// Indicates whether the shape is sliding down a slope after its kinematic movement.
    pub is_sliding_down_slope: bool,
}

/// The allowed movement computed by `RapierContext::move_shape`.
pub struct MoveShapeOutput {
    /// Indicates whether the shape is grounded after its kinematic movement.
    pub grounded: bool,
    /// The translation calculated by the last character control step taking obstacles into account.
    pub effective_translation: Vect,
    /// Indicates whether the shape is sliding down a slope after its kinematic movement.
    pub is_sliding_down_slope: bool,
}
