use crate::math::Vect;
use rapier::parry::shape::ConvexPolygon;

/// Read-only access to the properties of a convex polygon.
pub struct ConvexPolygonView<'a> {
    /// The raw shape from Rapier.
    pub raw: &'a ConvexPolygon,
}

impl<'a> ConvexPolygonView<'a> {
    /// The vertices of this convex polygon.
    pub fn points(&self) -> impl ExactSizeIterator<Item = Vect> + '_ {
        self.raw.points().iter().map(|pt| (*pt).into())
    }

    /// The normals of the edges of this convex polygon.
    pub fn normals(&self) -> impl ExactSizeIterator<Item = Vect> + '_ {
        self.raw.normals().iter().map(|n| (**n).into())
    }
}

/// Read-write access to the properties of a convex polygon.
pub struct ConvexPolygonViewMut<'a> {
    /// The raw shape from Rapier.
    pub raw: &'a mut ConvexPolygon,
}

impl<'a> ConvexPolygonViewMut<'a> {
    /// The vertices of this convex polygon.
    pub fn points(&self) -> impl ExactSizeIterator<Item = Vect> + '_ {
        self.raw.points().iter().map(|pt| (*pt).into())
    }

    /// The normals of the edges of this convex polygon.
    pub fn normals(&self) -> impl ExactSizeIterator<Item = Vect> + '_ {
        self.raw.normals().iter().map(|n| (**n).into())
    }

    // TODO: add modifications.
}
