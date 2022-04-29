use crate::math::{Real, Vect};
use rapier::parry::shape::HeightField;
pub use rapier::parry::shape::HeightFieldCellStatus;

/// Read-only access to the properties of a heightfield.
pub struct HeightFieldView<'a> {
    /// The raw shape from Rapier.
    pub raw: &'a HeightField,
}

macro_rules! impl_ref_methods(
    ($View: ident) => {
        #[cfg(feature = "dim2")]
        impl<'a> $View<'a> {
            /// The number of cells of this heightfield.
            pub fn num_cells(&self) -> usize {
                self.raw.num_cells()
            }

            /// The height at each cell endpoint.
            pub fn heights(&self) -> &[Real] {
                self.raw.heights().as_slice()
            }

            /// The scale factor applied to this heightfield.
            pub fn scale(&self) -> Vect {
                (*self.raw.scale()).into()
            }

            /// The width of a single cell of this heightfield.
            pub fn cell_width(&self) -> Real {
                self.raw.cell_width()
            }

            /// The width of a single cell of this heightfield, without taking the scale factor into account.
            pub fn unit_cell_width(&self) -> Real {
                self.raw.unit_cell_width()
            }

            /// Index of the cell a point is on after vertical projection.
            pub fn cell_at_point(&self, pt: Vect) -> Option<usize> {
                self.raw.cell_at_point(&pt.into())
            }

            /// Iterator through all the segments of this heightfield.
            pub fn segments(&self) -> impl Iterator<Item = (Vect, Vect)> + '_ {
                self.raw.segments().map(|seg| (seg.a.into(), seg.b.into()))
            }

            /// The i-th segment of the heightfield if it has not been removed.
            pub fn segment_at(&self, i: usize) -> Option<(Vect, Vect)> {
                self.raw
                    .segment_at(i)
                    .map(|seg| (seg.a.into(), seg.b.into()))
            }

            /// Is the i-th segment of this heightfield removed?
            pub fn is_segment_removed(&self, i: usize) -> bool {
                self.raw.is_segment_removed(i)
            }
        }

        #[cfg(feature = "dim3")]
        impl<'a> $View<'a> {
            /// The number of rows of this heightfield.
            pub fn nrows(&self) -> usize {
                self.raw.nrows()
            }

            /// The number of columns of this heightfield.
            pub fn ncols(&self) -> usize {
                self.raw.ncols()
            }

            /// The height at each cell endpoint.
            pub fn heights(&self) -> &[Real] {
                self.raw.heights().as_slice()
            }

            /// The scale factor applied to this heightfield.
            pub fn scale(&self) -> Vect {
                (*self.raw.scale()).into()
            }

            /// The width (extent along its local `x` axis) of each cell of this heightmap, including the scale factor.
            pub fn cell_width(&self) -> Real {
                self.raw.cell_width()
            }

            /// The height (extent along its local `z` axis) of each cell of this heightmap, including the scale factor.
            pub fn cell_height(&self) -> Real {
                self.raw.cell_height()
            }

            /// The width (extent along its local `x` axis) of each cell of this heightmap, excluding the scale factor.
            pub fn unit_cell_width(&self) -> Real {
                self.raw.unit_cell_width()
            }

            /// The height (extent along its local `z` axis) of each cell of this heightmap, excluding the scale factor.
            pub fn unit_cell_height(&self) -> Real {
                self.raw.unit_cell_height()
            }

            /// Index of the cell a point is on after vertical projection.
            pub fn cell_at_point(&self, pt: Vect) -> Option<(usize, usize)> {
                self.raw.cell_at_point(&pt.into())
            }

            /// Iterator through all the triangles of this heightfield.
            pub fn triangles(&self) -> impl Iterator<Item = (Vect, Vect, Vect)> + '_ {
                self.raw
                    .triangles()
                    .map(|tri| (tri.a.into(), tri.b.into(), tri.c.into()))
            }

            /// The two triangles at the cell (i, j) of this heightfield.
            ///
            /// Returns `None` fore triangles that have been removed because of their user-defined status
            /// flags (described by the `HeightFieldCellStatus` bitfield).
            pub fn triangles_at(
                &self,
                i: usize,
                j: usize,
            ) -> (Option<(Vect, Vect, Vect)>, Option<(Vect, Vect, Vect)>) {
                let (tri1, tri2) = self.raw.triangles_at(i, j);
                (
                    tri1.map(|tri| (tri.a.into(), tri.b.into(), tri.c.into())),
                    tri2.map(|tri| (tri.a.into(), tri.b.into(), tri.c.into())),
                )
            }

            /// The status of the `(i, j)`-th cell.
            pub fn cell_status(&self, i: usize, j: usize) -> HeightFieldCellStatus {
                self.raw.cell_status(i, j)
            }

            /// The statuses of all the cells of this heightfield, in column-major order.
            pub fn cells_statuses(&self) -> &[HeightFieldCellStatus] {
                self.raw.cells_statuses().as_slice()
            }
        }
    }
);

impl_ref_methods!(HeightFieldView);

/// Read-write access to the properties of a heightfield.
pub struct HeightFieldViewMut<'a> {
    /// The raw shape from Rapier.
    pub raw: &'a mut HeightField,
}

impl_ref_methods!(HeightFieldViewMut);

#[cfg(feature = "dim2")]
impl<'a> HeightFieldViewMut<'a> {
    /// Sets whether or not the given cell of the heightfield is deleted.
    pub fn set_segment_removed(&mut self, i: usize, removed: bool) {
        self.raw.set_segment_removed(i, removed)
    }
}

#[cfg(feature = "dim3")]
impl<'a> HeightFieldViewMut<'a> {
    /// Set the status of the `(i, j)`-th cell.
    pub fn set_cell_status(&mut self, i: usize, j: usize, status: HeightFieldCellStatus) {
        self.raw.set_cell_status(i, j, status)
    }

    /// The mutable statuses of all the cells of this heightfield.
    pub fn cells_statuses_mut(&mut self) -> &mut [HeightFieldCellStatus] {
        self.raw.cells_statuses_mut().as_mut_slice()
    }
}
