use crate::{
    parry::{
        self,
        query::{Contact, Unsupported},
        shape::Shape,
    },
    prelude::*,
    utils::pos_rot_to_iso,
};

/// Results from [`closest_points`]
pub enum ClosestPoints {
    /// The two objects are intersecting.
    Intersecting,
    /// The two objects are non-intersecting but closer than a given user-defined distance.
    WithinMargin(Vect, Vect),
    /// The two objects are non-intersecting and further than a given user-defined distance.
    Disjoint,
}

use parry::query::closest_points::ClosestPoints as ParryClosestPoints;
impl From<ParryClosestPoints> for ClosestPoints {
    fn from(parry: ParryClosestPoints) -> ClosestPoints {
        match parry {
            ParryClosestPoints::Intersecting => ClosestPoints::Intersecting,
            ParryClosestPoints::Disjoint => ClosestPoints::Disjoint,
            ParryClosestPoints::WithinMargin(p1, p2) => {
                ClosestPoints::WithinMargin(p1.into(), p2.into())
            }
        }
    }
}

/// Computes the pair of closest points between two shapes.
///
/// Returns `ClosestPoints::Disjoint` if the objects are separated by a distance greater than `max_dist`. The result points in `ClosestPoints::WithinMargin` are expressed in world-space.
pub fn closest_points(
    pos1: Vect,
    rot1: Rot,
    shape1: &dyn Shape,
    pos2: Vect,
    rot2: Rot,
    shape2: &dyn Shape,
    max_dist: Real,
    physics_scale: Real,
) -> Result<ClosestPoints, Unsupported> {
    let iso1 = pos_rot_to_iso(pos1, rot1, physics_scale);
    let iso2 = pos_rot_to_iso(pos2, rot2, physics_scale);

    parry::query::closest_points(&iso1, shape1, &iso2, shape2, max_dist).map(|parry| parry.into())
}

/// Computes the minimum distance separating two shapes.
///
/// Returns 0.0 if the objects are touching or penetrating.
pub fn distance(
    pos1: Vect,
    rot1: Rot,
    shape1: &dyn Shape,
    pos2: Vect,
    rot2: Rot,
    shape2: &dyn Shape,
    physics_scale: Real,
) -> Result<Real, Unsupported> {
    let iso1 = pos_rot_to_iso(pos1, rot1, physics_scale);
    let iso2 = pos_rot_to_iso(pos2, rot2, physics_scale);

    parry::query::distance(&iso1, shape1, &iso2, shape2)
}

/// Computes one pair of contact points point between two shapes.
///
/// Returns None if the objects are separated by a distance greater than prediction. The result is given in world-space.
pub fn contact(
    pos1: Vect,
    rot1: Rot,
    shape1: &dyn Shape,
    pos2: Vect,
    rot2: Rot,
    shape2: &dyn Shape,
    prediction: Real,
    physics_scale: Real,
) -> Result<Option<Contact>, Unsupported> {
    let iso1 = pos_rot_to_iso(pos1, rot1, physics_scale);
    let iso2 = pos_rot_to_iso(pos2, rot2, physics_scale);

    parry::query::contact(&iso1, shape1, &iso2, shape2, prediction)
}

/// Tests whether two shapes are intersecting.
pub fn intersection_test(
    pos1: Vect,
    rot1: Rot,
    shape1: &dyn Shape,
    pos2: Vect,
    rot2: Rot,
    shape2: &dyn Shape,
    physics_scale: Real,
) -> Result<bool, Unsupported> {
    let iso1 = pos_rot_to_iso(pos1, rot1, physics_scale);
    let iso2 = pos_rot_to_iso(pos2, rot2, physics_scale);

    parry::query::intersection_test(&iso1, shape1, &iso2, shape2)
}
