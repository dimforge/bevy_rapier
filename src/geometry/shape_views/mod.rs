pub use self::{
    ball::*, capsule::*, collider_view::*, compound::*, cuboid::*, halfspace::*, heightfield::*,
    polyline::*, round_shape::*, segment::*, triangle::*, trimesh::*,
};

#[cfg(feature = "dim2")]
pub use self::convex_polygon::*;

#[cfg(feature = "dim3")]
pub use self::{cone::*, convex_polyhedron::*, cylinder::*};

mod ball;
mod capsule;
mod collider_view;
mod compound;
mod cuboid;
mod halfspace;
mod heightfield;
mod polyline;
mod round_shape;
mod segment;
mod triangle;
mod trimesh;

#[cfg(feature = "dim2")]
mod convex_polygon;

#[cfg(feature = "dim3")]
mod cone;
#[cfg(feature = "dim3")]
mod convex_polyhedron;
#[cfg(feature = "dim3")]
mod cylinder;
