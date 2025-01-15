pub use self::collider::*;
pub use self::shape_views::ColliderView;
pub use rapier::geometry::SolverFlags;
pub use rapier::parry::query::{ShapeCastOptions, ShapeCastStatus};
pub use rapier::parry::shape::TriMeshFlags;
pub use rapier::parry::transformation::{vhacd::VHACDParameters, voxelization::FillMode};

use crate::math::{Real, Vect};
use rapier::prelude::FeatureId;

mod collider;
mod collider_impl;
/// Wrappers around Rapier shapes to access their properties.
pub mod shape_views;
#[cfg(feature = "collider-to-bevy-mesh")]
pub mod to_bevy_mesh;

/// Result of the projection of a point on a shape.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct PointProjection {
    /// Whether or not the point to project was inside of the shape.
    pub is_inside: bool,
    /// The projection result.
    pub point: Vect,
}

impl PointProjection {
    pub(crate) fn from_rapier(raw: rapier::parry::query::PointProjection) -> Self {
        Self {
            is_inside: raw.is_inside,
            point: raw.point.into(),
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
    /// with `origin + dir * time_of_impact` where `origin` is the origin of the ray;
    /// `dir` is its direction and `time_of_impact` is the value of this field.
    pub time_of_impact: Real,

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
            time_of_impact: inter.time_of_impact,
            point: unscaled_origin + unscaled_dir * inter.time_of_impact,
            normal: inter.normal.into(),
            feature: inter.feature,
        }
    }
}

/// The result of a shape cast.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct ShapeCastHit {
    /// The time at which the objects touch.
    pub time_of_impact: Real,
    /// Detail about the impact points.
    ///
    /// `None` if `status` is `PenetratingOrWithinTargetDist` and
    /// [`ShapeCastOptions::compute_impact_geometry_on_penetration`] was `false`.
    pub details: Option<ShapeCastHitDetails>,
    /// The way the time-of-impact computation algorithm terminated.
    pub status: ShapeCastStatus,
}

/// In depth information about a shape-cast hit.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct ShapeCastHitDetails {
    /// The local-space closest point on the first shape at the time of impact.
    pub witness1: Vect,
    /// The local-space closest point on the second shape at the time of impact.
    pub witness2: Vect,
    /// The local-space outward normal on the first shape at the time of impact.
    pub normal1: Vect,
    /// The local-space outward normal on the second shape at the time of impact.
    pub normal2: Vect,
}

impl ShapeCastHit {
    /// Convert from internal `rapier::query::ShapeCastHit`.
    pub fn from_rapier(
        hit: rapier::parry::query::ShapeCastHit,
        details_always_computed: bool,
    ) -> Self {
        let details = match (details_always_computed, hit.status) {
            (_, ShapeCastStatus::Failed) => None,
            (false, ShapeCastStatus::PenetratingOrWithinTargetDist) => None,
            _ => Some(ShapeCastHitDetails {
                witness1: hit.witness1.into(),
                witness2: hit.witness2.into(),
                normal1: hit.normal1.into(),
                normal2: hit.normal2.into(),
            }),
        };
        Self {
            time_of_impact: hit.time_of_impact,
            status: hit.status,
            details,
        }
    }
}
