use std::fmt;

use super::*;
use crate::math::Vect;
use rapier::geometry::{Shape, SharedShape};
use rapier::parry::shape::TypedShape;

/// Read-only access to the properties of a collider.
#[derive(Copy, Clone)]
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
    /// A custom user-defined shape with a type identified by a number.
    Custom(&'a dyn Shape),
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
            ColliderView::Custom(_) => write!(f, "Custom"),
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
            TypedShape::Custom(i) => ColliderView::Custom(i),
        }
    }
}

impl<'a> From<ColliderView<'a>> for TypedShape<'a> {
    fn from(collider_view: ColliderView<'a>) -> TypedShape<'a> {
        collider_view.as_typed_shape()
    }
}

impl<'a> From<ColliderView<'a>> for SharedShape {
    fn from(collider_view: ColliderView<'a>) -> SharedShape {
        collider_view.to_shared_shape()
    }
}

impl<'a> ColliderView<'a> {
    /// Convert to [`parry::TypedShape`].
    pub fn as_typed_shape(self) -> TypedShape<'a> {
        match self {
            ColliderView::Ball(BallView { raw: s }) => TypedShape::Ball(s),
            ColliderView::Cuboid(CuboidView { raw: s }) => TypedShape::Cuboid(s),
            ColliderView::Capsule(CapsuleView { raw: s }) => TypedShape::Capsule(s),
            ColliderView::Segment(SegmentView { raw: s }) => TypedShape::Segment(s),
            ColliderView::Triangle(TriangleView { raw: s }) => TypedShape::Triangle(s),
            ColliderView::TriMesh(TriMeshView { raw: s }) => TypedShape::TriMesh(s),
            ColliderView::Polyline(PolylineView { raw: s }) => TypedShape::Polyline(s),
            ColliderView::HalfSpace(HalfSpaceView { raw: s }) => TypedShape::HalfSpace(s),
            ColliderView::HeightField(HeightFieldView { raw: s }) => TypedShape::HeightField(s),
            ColliderView::Compound(CompoundView { raw: s }) => TypedShape::Compound(s),
            #[cfg(feature = "dim2")]
            ColliderView::ConvexPolygon(ConvexPolygonView { raw: s }) => {
                TypedShape::ConvexPolygon(s)
            }
            #[cfg(feature = "dim3")]
            ColliderView::ConvexPolyhedron(ConvexPolyhedronView { raw: s }) => {
                TypedShape::ConvexPolyhedron(s)
            }
            #[cfg(feature = "dim3")]
            ColliderView::Cylinder(CylinderView { raw: s }) => TypedShape::Cylinder(s),
            #[cfg(feature = "dim3")]
            ColliderView::Cone(ConeView { raw: s }) => TypedShape::Cone(s),
            ColliderView::RoundCuboid(RoundCuboidView { raw: s }) => TypedShape::RoundCuboid(s),
            ColliderView::RoundTriangle(RoundTriangleView { raw: s }) => {
                TypedShape::RoundTriangle(s)
            }
            // RoundedTriMesh,
            // RoundedHeightField,
            #[cfg(feature = "dim2")]
            ColliderView::RoundConvexPolygon(RoundConvexPolygonView { raw: s }) => {
                TypedShape::RoundConvexPolygon(s)
            }
            #[cfg(feature = "dim3")]
            ColliderView::RoundCylinder(RoundCylinderView { raw: s }) => {
                TypedShape::RoundCylinder(s)
            }
            #[cfg(feature = "dim3")]
            ColliderView::RoundCone(RoundConeView { raw: s }) => TypedShape::RoundCone(s),
            #[cfg(feature = "dim3")]
            ColliderView::RoundConvexPolyhedron(RoundConvexPolyhedronView { raw: s }) => {
                TypedShape::RoundConvexPolyhedron(s)
            }
            ColliderView::Custom(s) => TypedShape::Custom(s),
        }
    }

    /// Convert to [`parry::SharedShape`].
    pub fn to_shared_shape(self) -> SharedShape {
        match self {
            ColliderView::Ball(BallView { raw }) => SharedShape::new(*raw),
            ColliderView::Cuboid(CuboidView { raw }) => SharedShape::new(*raw),
            ColliderView::Capsule(CapsuleView { raw }) => SharedShape::new(*raw),
            ColliderView::Segment(SegmentView { raw }) => SharedShape::new(*raw),
            ColliderView::Triangle(TriangleView { raw }) => SharedShape::new(*raw),
            ColliderView::TriMesh(TriMeshView { raw }) => SharedShape::new(raw.clone()),
            ColliderView::Polyline(PolylineView { raw }) => SharedShape::new(raw.clone()),
            ColliderView::HalfSpace(HalfSpaceView { raw }) => SharedShape::new(*raw),
            ColliderView::HeightField(HeightFieldView { raw }) => SharedShape::new(raw.clone()),
            ColliderView::Compound(CompoundView { raw }) => SharedShape::new(raw.clone()),
            #[cfg(feature = "dim2")]
            ColliderView::ConvexPolygon(ConvexPolygonView { raw }) => SharedShape::new(raw.clone()),
            #[cfg(feature = "dim3")]
            ColliderView::ConvexPolyhedron(ConvexPolyhedronView { raw }) => {
                SharedShape::new(raw.clone())
            }
            #[cfg(feature = "dim3")]
            ColliderView::Cylinder(CylinderView { raw }) => SharedShape::new(*raw),
            #[cfg(feature = "dim3")]
            ColliderView::Cone(ConeView { raw }) => SharedShape::new(*raw),
            ColliderView::RoundCuboid(RoundCuboidView { raw }) => SharedShape::new(*raw),
            ColliderView::RoundTriangle(RoundTriangleView { raw }) => SharedShape::new(*raw),
            #[cfg(feature = "dim2")]
            ColliderView::RoundConvexPolygon(RoundConvexPolygonView { raw }) => {
                SharedShape::new(raw.clone())
            }
            #[cfg(feature = "dim3")]
            ColliderView::RoundCylinder(RoundCylinderView { raw }) => SharedShape::new(*raw),
            #[cfg(feature = "dim3")]
            ColliderView::RoundCone(RoundConeView { raw }) => SharedShape::new(*raw),
            #[cfg(feature = "dim3")]
            ColliderView::RoundConvexPolyhedron(RoundConvexPolyhedronView { raw }) => {
                SharedShape::new(raw.clone())
            }
            ColliderView::Custom(i) => SharedShape(i.clone_box().into()),
        }
    }

    pub fn to_shape(&self) -> &dyn Shape {
        match self {
            ColliderView::Ball(BallView { raw }) => *raw,
            ColliderView::Cuboid(CuboidView { raw }) => *raw,
            ColliderView::Capsule(CapsuleView { raw }) => *raw,
            ColliderView::Segment(SegmentView { raw }) => *raw,
            ColliderView::Triangle(TriangleView { raw }) => *raw,
            ColliderView::TriMesh(TriMeshView { raw }) => *raw,
            ColliderView::Polyline(PolylineView { raw }) => *raw,
            ColliderView::HalfSpace(HalfSpaceView { raw }) => *raw,
            ColliderView::HeightField(HeightFieldView { raw }) => *raw,
            ColliderView::Compound(CompoundView { raw }) => *raw,
            #[cfg(feature = "dim2")]
            ColliderView::ConvexPolygon(ConvexPolygonView { raw }) => *raw,
            #[cfg(feature = "dim3")]
            ColliderView::ConvexPolyhedron(ConvexPolyhedronView { raw }) => *raw,
            #[cfg(feature = "dim3")]
            ColliderView::Cylinder(CylinderView { raw }) => *raw,
            #[cfg(feature = "dim3")]
            ColliderView::Cone(ConeView { raw }) => *raw,
            ColliderView::RoundCuboid(RoundCuboidView { raw }) => *raw,
            ColliderView::RoundTriangle(RoundTriangleView { raw }) => *raw,
            #[cfg(feature = "dim2")]
            ColliderView::RoundConvexPolygon(RoundConvexPolygonView { raw }) => *raw,
            #[cfg(feature = "dim3")]
            ColliderView::RoundCylinder(RoundCylinderView { raw }) => *raw,
            #[cfg(feature = "dim3")]
            ColliderView::RoundCone(RoundConeView { raw }) => *raw,
            #[cfg(feature = "dim3")]
            ColliderView::RoundConvexPolyhedron(RoundConvexPolyhedronView { raw }) => *raw,
            ColliderView::Custom(raw) => *raw,
        }
    }

    /// Compute the scaled version of `self.raw`.
    pub fn raw_scale_by(&self, scale: Vect, num_subdivisions: u32) -> Option<SharedShape> {
        let shape = self
            .to_shape()
            .clone_scaled(&scale.into(), num_subdivisions)?;
        Some(SharedShape(shape.into()))
    }
}
