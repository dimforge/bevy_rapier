use crate::math::Vect;
use rapier::geometry::{Cuboid, RoundCuboid};

pub struct CuboidView<'a> {
    pub raw: &'a Cuboid,
}

macro_rules! impl_ref_methods(
    ($View: ident) => {
        impl<'a> $View<'a> {
            /// The half-extents of the cuboid.
            pub fn half_extents(&self) -> Vect {
                self.raw.half_extents.into()
            }
        }
    }
);

impl_ref_methods!(CuboidView);

pub struct CuboidViewMut<'a> {
    pub raw: &'a mut Cuboid,
}

impl_ref_methods!(CuboidViewMut);

impl<'a> CuboidViewMut<'a> {
    /// Set the half-extents of the cuboid.
    pub fn sed_half_extents(&mut self, half_extents: Vect) {
        self.raw.half_extents = half_extents.into();
    }
}
