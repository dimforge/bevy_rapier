use crate::math::Vect;
use rapier::parry::shape::{TopologyError, TriMesh, TriMeshFlags};

pub struct TriMeshView<'a> {
    pub raw: &'a TriMesh,
}

macro_rules! impl_ref_methods(
    ($View: ident) => {
        impl<'a> $View<'a> {
            /// The number of triangles forming this mesh.
            pub fn num_triangles(&self) -> usize {
                self.raw.num_triangles()
            }

            /// An iterator through all the triangles of this mesh.
            pub fn triangles(&self) -> impl ExactSizeIterator<Item = (Vect, Vect, Vect)> + '_ {
                self.raw
                    .triangles()
                    .map(|tri| (tri.a.into(), tri.b.into(), tri.c.into()))
            }

            /// Get the `i`-th triangle of this mesh.
            pub fn triangle(&self, i: u32) -> (Vect, Vect, Vect) {
                let tri = self.raw.triangle(i);
                (tri.a.into(), tri.b.into(), tri.c.into())
            }

            /// The vertex buffer of this mesh.
            pub fn vertices(&self) -> impl ExactSizeIterator<Item = Vect> + '_ {
                self.raw.vertices().iter().map(|pt| (*pt).into())
            }

            /// The index buffer of this mesh.
            pub fn indices(&self) -> &[[u32; 3]] {
                self.raw.indices()
            }

            /// A flat view of the index buffer of this mesh.
            pub fn flat_indices(&self) -> &[u32] {
                self.raw.flat_indices()
            }
        }
    }
);

impl_ref_methods!(TriMeshView);

pub struct TriMeshViewMut<'a> {
    pub raw: &'a mut TriMesh,
}

impl_ref_methods!(TriMeshViewMut);

impl<'a> TriMeshViewMut<'a> {
    pub fn set_flags(&mut self, flags: TriMeshFlags) -> Result<(), TopologyError> {
        self.raw.set_flags(flags)
    }
}
