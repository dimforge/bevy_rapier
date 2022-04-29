use crate::math::Vect;
use rapier::parry::shape::ConvexPolyhedron;

/// Read-only access to the properties of a convex polyhedron.
pub struct ConvexPolyhedronView<'a> {
    /// The raw shape from Rapier.
    pub raw: &'a ConvexPolyhedron,
}

impl<'a> ConvexPolyhedronView<'a> {
    /// The vertices of this convex polygon.
    pub fn points(&self) -> impl ExactSizeIterator<Item = Vect> + '_ {
        self.raw.points().iter().map(|pt| (*pt).into())
    }

    // TODO: add retrieval of topology information.
}

/// Read-write access to the properties of a convex polyhedron.
pub struct ConvexPolyhedronViewMut<'a> {
    /// The raw shape from Rapier.
    pub raw: &'a mut ConvexPolyhedron,
}

impl<'a> ConvexPolyhedronViewMut<'a> {
    /// The vertices of this convex polygon.
    pub fn points(&self) -> impl ExactSizeIterator<Item = Vect> + '_ {
        self.raw.points().iter().map(|pt| (*pt).into())
    }

    // TODO: add retrieval of topology information and modification.
}
