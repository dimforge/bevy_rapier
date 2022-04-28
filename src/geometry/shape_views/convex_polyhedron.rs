use crate::math::Vect;
use rapier::parry::shape::ConvexPolyhedron;

pub struct ConvexPolyhedronView<'a> {
    pub raw: &'a ConvexPolyhedron,
}

impl<'a> ConvexPolyhedronView<'a> {
    /// The vertices of this convex polygon.
    pub fn points(&self) -> impl ExactSizeIterator<Item = Vect> + '_ {
        self.raw.points().iter().map(|pt| (*pt).into())
    }

    // TODO: add retrieval of topology information.
}

pub struct ConvexPolyhedronViewMut<'a> {
    pub raw: &'a mut ConvexPolyhedron,
}

impl<'a> ConvexPolyhedronViewMut<'a> {
    /// The vertices of this convex polygon.
    pub fn points(&self) -> impl ExactSizeIterator<Item = Vect> + '_ {
        self.raw.points().iter().map(|pt| (*pt).into())
    }

    // TODO: add retrieval of topology information and modification.
}
