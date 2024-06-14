use crate::math::Vect;
use rapier::parry::shape::{TopologyError, TriMesh, TriMeshFlags};

/// Read-only access to the properties of a triangle mesh.
#[derive(Copy, Clone)]
pub struct TriMeshView<'a> {
    /// The raw shape from Rapier.
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

/// Read-write access to the properties of a triangle mesh.
pub struct TriMeshViewMut<'a> {
    /// The raw shape from Rapier.
    pub raw: &'a mut TriMesh,
}

impl_ref_methods!(TriMeshViewMut);

impl<'a> TriMeshViewMut<'a> {
    /// Sets the flags of this triangle mesh, controlling its optional associated data.
    pub fn set_flags(&mut self, flags: TriMeshFlags) -> Result<(), TopologyError> {
        self.raw.set_flags(flags)
    }
}
