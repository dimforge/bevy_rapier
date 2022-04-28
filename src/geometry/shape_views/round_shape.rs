use crate::geometry::shape_views::{CuboidView, CuboidViewMut, TriangleView, TriangleViewMut};
use rapier::geometry::{RoundCuboid, RoundTriangle};

#[cfg(feature = "dim2")]
use {
    crate::geometry::shape_views::{ConvexPolygonView, ConvexPolygonViewMut},
    rapier::geometry::RoundConvexPolygon,
};

#[cfg(feature = "dim3")]
use {
    crate::geometry::shape_views::{
        ConeView, ConeViewMut, ConvexPolyhedronView, ConvexPolyhedronViewMut, CylinderView,
        CylinderViewMut,
    },
    rapier::geometry::{RoundCone, RoundConvexPolyhedron, RoundCylinder},
};

macro_rules!  round_shape_view(
    ($RoundShape: ident, $RoundShapeView: ident, $ShapeView: ident, $RoundShapeViewMut: ident, $ShapeViewMut: ident) => {
        pub struct $RoundShapeView<'a> {
            pub raw: &'a $RoundShape,
        }

        impl<'a> $RoundShapeView<'a> {
            pub fn border_radius(&self) -> f32 {
                self.raw.border_radius
            }

            pub fn inner_shape(&self) -> $ShapeView {
                $ShapeView {
                    raw: &self.raw.inner_shape,
                }
            }
        }

        pub struct $RoundShapeViewMut<'a> {
            pub raw: &'a mut $RoundShape,
        }

        impl<'a> $RoundShapeViewMut<'a> {
            pub fn border_radius(&self) -> f32 {
                self.raw.border_radius
            }

            pub fn set_border_radius(&mut self, new_border_radius: f32) {
                self.raw.border_radius = new_border_radius;
            }

            pub fn inner_shape(&mut self) -> $ShapeViewMut {
                $ShapeViewMut {
                    raw: &mut self.raw.inner_shape,
                }
            }
        }
    }
);

round_shape_view!(
    RoundCuboid,
    RoundCuboidView,
    CuboidView,
    RoundCuboidViewMut,
    CuboidViewMut
);

round_shape_view!(
    RoundTriangle,
    RoundTriangleView,
    TriangleView,
    RoundTriangleViewMut,
    TriangleViewMut
);

#[cfg(feature = "dim2")]
round_shape_view!(
    RoundConvexPolygon,
    RoundConvexPolygonView,
    ConvexPolygonView,
    RoundConvexPolygonViewMut,
    ConvexPolygonViewMut
);

#[cfg(feature = "dim3")]
round_shape_view!(
    RoundCone,
    RoundConeView,
    ConeView,
    RoundConeViewMut,
    ConeViewMut
);

#[cfg(feature = "dim3")]
round_shape_view!(
    RoundCylinder,
    RoundCylinderView,
    CylinderView,
    RoundCylinderViewMut,
    CylinderViewMut
);

#[cfg(feature = "dim3")]
round_shape_view!(
    RoundConvexPolyhedron,
    RoundConvexPolyhedronView,
    ConvexPolyhedronView,
    RoundConvexPolyhedronViewMut,
    ConvexPolyhedronViewMut
);
