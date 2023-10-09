pub use self::collider::*;
pub use self::shape_views::ColliderView;
pub use rapier::geometry::SolverFlags;
pub use rapier::parry::query::TOIStatus;
pub use rapier::parry::shape::TriMeshFlags;
pub use rapier::parry::transformation::{vhacd::VHACDParameters, voxelization::FillMode};

use crate::math::{Real, Vect};
use rapier::prelude::FeatureId;

mod collider;
mod collider_impl;
/// Wrappers around Parry shape queries.
pub mod query;
/// Wrappers around Rapier shapes to access their properties.
pub mod shape_views;

/// Result of the projection of a point on a shape.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct PointProjection {
    /// Whether or not the point to project was inside of the shape.
    pub is_inside: bool,
    /// The projection result.
    pub point: Vect,
}

impl PointProjection {
    pub(crate) fn from_rapier(
        physics_scale: Real,
        raw: rapier::parry::query::PointProjection,
    ) -> Self {
        Self {
            is_inside: raw.is_inside,
            point: (raw.point * physics_scale).into(),
        }
    }
}
impl From<rapier::parry::query::PointProjection> for PointProjection {
    fn from(projection: rapier::parry::query::PointProjection) -> PointProjection {
        PointProjection {
            is_inside: projection.is_inside,
            point: projection.point.into(),
        }
    }
}

/// Structure containing the result of a successful ray cast.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct RayIntersection {
    /// The time of impact of the ray with the object.  The exact contact point can be computed
    /// with `origin + dir * toi` where `origin` is the origin of the ray;
    /// `dir` is its direction and `toi` is the value of this field.
    pub toi: Real,

    /// The intersection point between the ray and the object.
    pub point: Vect,

    /// The normal at the intersection point.
    ///
    /// If the `toi` is exactly zero, the normal might not be reliable.
    pub normal: Vect,

    /// Feature at the intersection point.
    pub feature: FeatureId,
}

impl RayIntersection {
    pub(crate) fn from_rapier(
        inter: rapier::parry::query::RayIntersection,
        unscaled_origin: Vect,
        unscaled_dir: Vect,
    ) -> Self {
        Self {
            toi: inter.toi,
            point: unscaled_origin + unscaled_dir * inter.toi,
            normal: inter.normal.into(),
            feature: inter.feature,
        }
    }
}

/// The result of a time-of-impact (TOI) computation.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Toi {
    /// The time at which the objects touch.
    pub toi: Real,
    /// Detail about the impact points.
    ///
    /// `None` if `status` is `Penetrating`.
    pub details: Option<ToiDetails>,
    /// The way the time-of-impact computation algorithm terminated.
    pub status: TOIStatus,
}

/// In depth information about a time-of-impact (TOI) computation.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct ToiDetails {
    /// The local-space closest point on the first shape at the time of impact.
    pub witness1: Vect,
    /// The local-space closest point on the second shape at the time of impact.
    pub witness2: Vect,
    /// The local-space outward normal on the first shape at the time of impact.
    pub normal1: Vect,
    /// The local-space outward normal on the second shape at the time of impact.
    pub normal2: Vect,
}

impl Toi {
    /// Convert from internal `rapier::Toi`.
    pub fn from_rapier(physics_scale: Real, toi: rapier::parry::query::TOI) -> Self {
        let details = if toi.status != TOIStatus::Penetrating {
            Some(ToiDetails {
                witness1: (toi.witness1 * physics_scale).into(),
                witness2: (toi.witness2 * physics_scale).into(),
                normal1: toi.normal1.into(),
                normal2: toi.normal2.into(),
            })
        } else {
            None
        };
        Self {
            toi: toi.toi,
            status: toi.status,
            details,
        }
    }
}
