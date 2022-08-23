use std::fmt;

use super::*;
use crate::math::Vect;
use rapier::geometry::{RoundShape, SharedShape};
use rapier::parry::either::Either;
use rapier::parry::shape::TypedShape;

/// Read-only access to the properties of a collider.
pub enum ColliderView<'a> {
    /// A ball shape.
    Ball(BallView<'a>),
    /// A cuboid shape.
    Cuboid(CuboidView<'a>),
    /// A capsule shape.
    Capsule(CapsuleView<'a>),
    /// A segment shape.
    Segment(SegmentView<'a>),
    /// A triangle shape.
    Triangle(TriangleView<'a>),
    /// A triangle mesh shape.
    TriMesh(TriMeshView<'a>),
    /// A set of segments.
    Polyline(PolylineView<'a>),
    /// A shape representing a full half-space.
    HalfSpace(HalfSpaceView<'a>),
    /// A heightfield shape.
    HeightField(HeightFieldView<'a>),
    /// A Compound shape.
    Compound(CompoundView<'a>),
    /// A convex polygon.
    #[cfg(feature = "dim2")]
    ConvexPolygon(ConvexPolygonView<'a>),
    /// A convex polyhedron.
    #[cfg(feature = "dim3")]
    ConvexPolyhedron(ConvexPolyhedronView<'a>),
    #[cfg(feature = "dim3")]
    /// A cylindrical shape.
    Cylinder(CylinderView<'a>),
    #[cfg(feature = "dim3")]
    /// A cone shape.
    Cone(ConeView<'a>),
    /// A cuboid with rounded corners.
    RoundCuboid(RoundCuboidView<'a>),
    /// A triangle with rounded corners.
    RoundTriangle(RoundTriangleView<'a>),
    // /// A triangle-mesh with rounded corners.
    // RoundedTriMesh,
    // /// An heightfield with rounded corners.
    // RoundedHeightField,
    /// A cylinder with rounded corners.
    #[cfg(feature = "dim3")]
    RoundCylinder(RoundCylinderView<'a>),
    /// A cone with rounded corners.
    #[cfg(feature = "dim3")]
    RoundCone(RoundConeView<'a>),
    /// A convex polyhedron with rounded corners.
    #[cfg(feature = "dim3")]
    RoundConvexPolyhedron(RoundConvexPolyhedronView<'a>),
    /// A convex polygon with rounded corners.
    #[cfg(feature = "dim2")]
    RoundConvexPolygon(RoundConvexPolygonView<'a>),
}
impl<'a> fmt::Debug for ColliderView<'a> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ColliderView::Ball(view) => write!(f, "{:?}", view.raw),
            ColliderView::Cuboid(view) => write!(f, "{:?}", view.raw),
            ColliderView::Capsule(view) => write!(f, "{:?}", view.raw),
            ColliderView::Segment(view) => write!(f, "{:?}", view.raw),
            ColliderView::Triangle(view) => write!(f, "{:?}", view.raw),
            ColliderView::TriMesh(_) => write!(f, "Trimesh (not representable)"),
            ColliderView::Polyline(_) => write!(f, "Polyline (not representable)"),
            ColliderView::HalfSpace(view) => write!(f, "{:?}", view.raw),
            ColliderView::HeightField(view) => write!(f, "{:?}", view.raw),
            ColliderView::Compound(_) => write!(f, "Compound (not representable)"),
            #[cfg(feature = "dim2")]
            ColliderView::ConvexPolygon(view) => write!(f, "{:?}", view.raw),
            #[cfg(feature = "dim3")]
            ColliderView::ConvexPolyhedron(view) => write!(f, "{:?}", view.raw),
            #[cfg(feature = "dim3")]
            ColliderView::Cylinder(view) => write!(f, "{:?}", view.raw),
            #[cfg(feature = "dim3")]
            ColliderView::Cone(view) => write!(f, "{:?}", view.raw),
            ColliderView::RoundCuboid(view) => write!(f, "{:?}", view.raw),
            ColliderView::RoundTriangle(view) => write!(f, "{:?}", view.raw),
            #[cfg(feature = "dim3")]
            ColliderView::RoundCylinder(view) => write!(f, "{:?}", view.raw),
            #[cfg(feature = "dim3")]
            ColliderView::RoundCone(view) => write!(f, "{:?}", view.raw),
            #[cfg(feature = "dim3")]
            ColliderView::RoundConvexPolyhedron(view) => write!(f, "{:?}", view.raw),
            #[cfg(feature = "dim2")]
            ColliderView::RoundConvexPolygon(view) => write!(f, "{:?}", view.raw),
        }
    }
}

impl<'a> From<TypedShape<'a>> for ColliderView<'a> {
    fn from(typed_shape: TypedShape<'a>) -> ColliderView<'a> {
        match typed_shape {
            TypedShape::Ball(s) => ColliderView::Ball(BallView { raw: s }),
            TypedShape::Cuboid(s) => ColliderView::Cuboid(CuboidView { raw: s }),
            TypedShape::Capsule(s) => ColliderView::Capsule(CapsuleView { raw: s }),
            TypedShape::Segment(s) => ColliderView::Segment(SegmentView { raw: s }),
            TypedShape::Triangle(s) => ColliderView::Triangle(TriangleView { raw: s }),
            TypedShape::TriMesh(s) => ColliderView::TriMesh(TriMeshView { raw: s }),
            TypedShape::Polyline(s) => ColliderView::Polyline(PolylineView { raw: s }),
            TypedShape::HalfSpace(s) => ColliderView::HalfSpace(HalfSpaceView { raw: s }),
            TypedShape::HeightField(s) => ColliderView::HeightField(HeightFieldView { raw: s }),
            TypedShape::Compound(s) => ColliderView::Compound(CompoundView { raw: s }),
            #[cfg(feature = "dim2")]
            TypedShape::ConvexPolygon(s) => {
                ColliderView::ConvexPolygon(ConvexPolygonView { raw: s })
            }
            #[cfg(feature = "dim3")]
            TypedShape::ConvexPolyhedron(s) => {
                ColliderView::ConvexPolyhedron(ConvexPolyhedronView { raw: s })
            }
            #[cfg(feature = "dim3")]
            TypedShape::Cylinder(s) => ColliderView::Cylinder(CylinderView { raw: s }),
            #[cfg(feature = "dim3")]
            TypedShape::Cone(s) => ColliderView::Cone(ConeView { raw: s }),
            TypedShape::RoundCuboid(s) => ColliderView::RoundCuboid(RoundCuboidView { raw: s }),
            TypedShape::RoundTriangle(s) => {
                ColliderView::RoundTriangle(RoundTriangleView { raw: s })
            }
            // RoundedTriMesh,
            // RoundedHeightField,
            #[cfg(feature = "dim2")]
            TypedShape::RoundConvexPolygon(s) => {
                ColliderView::RoundConvexPolygon(RoundConvexPolygonView { raw: s })
            }
            #[cfg(feature = "dim3")]
            TypedShape::RoundCylinder(s) => {
                ColliderView::RoundCylinder(RoundCylinderView { raw: s })
            }
            #[cfg(feature = "dim3")]
            TypedShape::RoundCone(s) => ColliderView::RoundCone(RoundConeView { raw: s }),
            #[cfg(feature = "dim3")]
            TypedShape::RoundConvexPolyhedron(s) => {
                ColliderView::RoundConvexPolyhedron(RoundConvexPolyhedronView { raw: s })
            }
            TypedShape::Custom(_) => todo!(),
        }
    }
}

impl<'a> ColliderView<'a> {
    /// Compute the scaled version of `self.raw`.
    pub fn raw_scale_by(&self, scale: Vect, num_subdivisions: u32) -> Option<SharedShape> {
        let result = match self {
            ColliderView::Cuboid(s) => SharedShape::new(s.raw.scaled(&scale.into())),
            ColliderView::RoundCuboid(s) => SharedShape::new(RoundShape {
                border_radius: s.raw.border_radius,
                inner_shape: s.raw.inner_shape.scaled(&scale.into()),
            }),
            ColliderView::Capsule(c) => match c.raw.scaled(&scale.into(), num_subdivisions) {
                None => {
                    log::error!("Failed to apply scale {} to Capsule shape.", scale);
                    SharedShape::ball(0.0)
                }
                Some(Either::Left(b)) => SharedShape::new(b),
                Some(Either::Right(b)) => SharedShape::new(b),
            },
            ColliderView::Ball(b) => match b.raw.scaled(&scale.into(), num_subdivisions) {
                None => {
                    log::error!("Failed to apply scale {} to Ball shape.", scale);
                    SharedShape::ball(0.0)
                }
                Some(Either::Left(b)) => SharedShape::new(b),
                Some(Either::Right(b)) => SharedShape::new(b),
            },
            ColliderView::Segment(s) => SharedShape::new(s.raw.scaled(&scale.into())),
            // ColliderView::RoundSegment(s) => SharedShape::new(RoundShape {
            //     border_radius: s.raw.border_radius,
            //     inner_shape: s.raw.inner_shape.scaled(&scale.into()),
            // }),
            ColliderView::Triangle(t) => SharedShape::new(t.raw.scaled(&scale.into())),
            ColliderView::RoundTriangle(t) => SharedShape::new(RoundShape {
                border_radius: t.raw.border_radius,
                inner_shape: t.raw.inner_shape.scaled(&scale.into()),
            }),
            ColliderView::TriMesh(t) => SharedShape::new(t.raw.clone().scaled(&scale.into())),
            ColliderView::Polyline(p) => SharedShape::new(p.raw.clone().scaled(&scale.into())),
            ColliderView::HalfSpace(h) => match h.raw.scaled(&scale.into()) {
                None => {
                    log::error!("Failed to apply scale {} to HalfSpace shape.", scale);
                    SharedShape::ball(0.0)
                }
                Some(scaled) => SharedShape::new(scaled),
            },
            ColliderView::HeightField(h) => SharedShape::new(h.raw.clone().scaled(&scale.into())),
            #[cfg(feature = "dim2")]
            ColliderView::ConvexPolygon(cp) => match cp.raw.clone().scaled(&scale.into()) {
                None => {
                    log::error!("Failed to apply scale {} to ConvexPolygon shape.", scale);
                    SharedShape::ball(0.0)
                }
                Some(scaled) => SharedShape::new(scaled),
            },
            #[cfg(feature = "dim2")]
            ColliderView::RoundConvexPolygon(cp) => {
                match cp.raw.inner_shape.clone().scaled(&scale.into()) {
                    None => {
                        log::error!(
                            "Failed to apply scale {} to RoundConvexPolygon shape.",
                            scale
                        );
                        SharedShape::ball(0.0)
                    }
                    Some(scaled) => SharedShape::new(RoundShape {
                        border_radius: cp.raw.border_radius,
                        inner_shape: scaled,
                    }),
                }
            }
            #[cfg(feature = "dim3")]
            ColliderView::ConvexPolyhedron(cp) => match cp.raw.clone().scaled(&scale.into()) {
                None => {
                    log::error!("Failed to apply scale {} to ConvexPolyhedron shape.", scale);
                    SharedShape::ball(0.0)
                }
                Some(scaled) => SharedShape::new(scaled),
            },
            #[cfg(feature = "dim3")]
            ColliderView::RoundConvexPolyhedron(cp) => {
                match cp.raw.clone().inner_shape.scaled(&scale.into()) {
                    None => {
                        log::error!(
                            "Failed to apply scale {} to RoundConvexPolyhedron shape.",
                            scale
                        );
                        SharedShape::ball(0.0)
                    }
                    Some(scaled) => SharedShape::new(RoundShape {
                        border_radius: cp.raw.border_radius,
                        inner_shape: scaled,
                    }),
                }
            }
            #[cfg(feature = "dim3")]
            ColliderView::Cylinder(c) => match c.raw.scaled(&scale.into(), num_subdivisions) {
                None => {
                    log::error!("Failed to apply scale {} to Cylinder shape.", scale);
                    SharedShape::ball(0.0)
                }
                Some(Either::Left(b)) => SharedShape::new(b),
                Some(Either::Right(b)) => SharedShape::new(b),
            },
            #[cfg(feature = "dim3")]
            ColliderView::RoundCylinder(c) => {
                match c.raw.inner_shape.scaled(&scale.into(), num_subdivisions) {
                    None => {
                        log::error!("Failed to apply scale {} to RoundCylinder shape.", scale);
                        SharedShape::ball(0.0)
                    }
                    Some(Either::Left(scaled)) => SharedShape::new(RoundShape {
                        border_radius: c.raw.border_radius,
                        inner_shape: scaled,
                    }),
                    Some(Either::Right(scaled)) => SharedShape::new(RoundShape {
                        border_radius: c.raw.border_radius,
                        inner_shape: scaled,
                    }),
                }
            }
            #[cfg(feature = "dim3")]
            ColliderView::Cone(c) => match c.raw.scaled(&scale.into(), num_subdivisions) {
                None => {
                    log::error!("Failed to apply scale {} to Cone shape.", scale);
                    SharedShape::ball(0.0)
                }
                Some(Either::Left(b)) => SharedShape::new(b),
                Some(Either::Right(b)) => SharedShape::new(b),
            },
            #[cfg(feature = "dim3")]
            ColliderView::RoundCone(c) => {
                match c.raw.inner_shape.scaled(&scale.into(), num_subdivisions) {
                    None => {
                        log::error!("Failed to apply scale {} to RoundCone shape.", scale);
                        SharedShape::ball(0.0)
                    }
                    Some(Either::Left(scaled)) => SharedShape::new(RoundShape {
                        border_radius: c.raw.border_radius,
                        inner_shape: scaled,
                    }),
                    Some(Either::Right(scaled)) => SharedShape::new(RoundShape {
                        border_radius: c.raw.border_radius,
                        inner_shape: scaled,
                    }),
                }
            }
            ColliderView::Compound(c) => {
                let mut scaled = Vec::with_capacity(c.shapes().len());

                for (tra, rot, shape) in c.shapes() {
                    scaled.push((
                        (tra * scale, rot).into(),
                        shape.raw_scale_by(scale, num_subdivisions)?,
                    ));
                }
                SharedShape::compound(scaled)
            }
        };

        Some(result)
    }
}
