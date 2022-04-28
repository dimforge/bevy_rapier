use crate::math::Vect;
use rapier::parry::shape::HalfSpace;

pub struct HalfSpaceView<'a> {
    pub raw: &'a HalfSpace,
}

macro_rules! impl_ref_methods(
    ($View: ident) => {
        impl<'a> $View<'a> {
            /// The halfspace planar boundary's outward normal.
            pub fn normal(&self) -> Vect {
                (*self.raw.normal).into()
            }
        }
    }
);

impl_ref_methods!(HalfSpaceView);

pub struct HalfSpaceViewMut<'a> {
    pub raw: &'a mut HalfSpace,
}

impl_ref_methods!(HalfSpaceViewMut);

impl<'a> HalfSpaceViewMut<'a> {
    /// Set the normal of the half-space.
    pub fn set_normal(&mut self, normal: Vect) {
        let normal: rapier::math::Vector<_> = normal.into();
        if let Some(unit_normal) = na::Unit::try_new(normal, 1.0e-6) {
            self.raw.normal = unit_normal;
        }
    }
}
