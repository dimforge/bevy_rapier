use crate::math::Vect;
use rapier::parry::shape::Polyline;

/// Read-only access to the properties of a polyline.
pub struct PolylineView<'a> {
    /// The raw shape from Rapier.
    pub raw: &'a Polyline,
}

macro_rules! impl_ref_methods(
    ($View: ident) => {
        impl<'a> $View<'a> {
            /// The number of segments forming this polyline.
            pub fn num_segments(&self) -> usize {
                self.raw.num_segments()
            }

            /// An iterator through all the segments of this mesh.
            pub fn segments(&self) -> impl ExactSizeIterator<Item = (Vect, Vect)> + '_ {
                self.raw.segments().map(|seg| (seg.a.into(), seg.b.into()))
            }

            /// Get the `i`-th segment of this mesh.
            pub fn segment(&self, i: u32) -> (Vect, Vect) {
                let seg = self.raw.segment(i);
                (seg.a.into(), seg.b.into())
            }

            /// The vertex buffer, containing all the vertices of this polyline.
            pub fn vertices(&self) -> impl ExactSizeIterator<Item = Vect> + '_ {
                self.raw.vertices().iter().map(|v| (*v).into())
            }

            /// The index buffer, describing all the segments of this polyline.
            pub fn indices(&self) -> &[[u32; 2]] {
                self.raw.indices()
            }
        }
    }
);

impl_ref_methods!(PolylineView);

/// Read-write access to the properties of a polyline.
pub struct PolylineViewMut<'a> {
    /// The raw shape from Rapier.
    pub raw: &'a mut Polyline,
}

impl_ref_methods!(PolylineViewMut);

impl<'a> PolylineViewMut<'a> {
    /// Reverse the orientation of this polyline by swapping the indices of all
    /// its segments and reverting its index buffer.
    pub fn reverse(&mut self) {
        self.raw.reverse()
    }
}
