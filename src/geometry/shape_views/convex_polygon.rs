use crate::math::Vect;
use rapier::parry::shape::ConvexPolygon;

pub struct ConvexPolygonView<'a> {
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

pub struct ConvexPolygonViewMut<'a> {
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
