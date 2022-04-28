use crate::math::Real;
use rapier::parry::shape::Cone;

pub struct ConeView<'a> {
    pub raw: &'a Cone,
}

macro_rules! impl_ref_methods(
    ($View: ident) => {
        impl<'a> $View<'a> {
            /// The half-height of the cone.
            pub fn half_height(&self) -> Real {
                self.raw.half_height
            }

            /// The base radius of the cone.
            pub fn radius(&self) -> Real {
                self.raw.radius
            }
        }
    }
);

impl_ref_methods!(ConeView);

pub struct ConeViewMut<'a> {
    pub raw: &'a mut Cone,
}

impl_ref_methods!(ConeViewMut);

impl<'a> ConeViewMut<'a> {
    /// Set the half-height of the cone.
    pub fn set_half_height(&mut self, half_height: Real) {
        self.raw.half_height = half_height;
    }

    /// Set the radius of the basis of the cone.
    pub fn set_radius(&mut self, radius: Real) {
        self.raw.radius = radius;
    }
}
