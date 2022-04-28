use crate::math::Real;
use rapier::parry::shape::Cylinder;

pub struct CylinderView<'a> {
    pub raw: &'a Cylinder,
}

macro_rules! impl_ref_methods(
    ($View: ident) => {
        impl<'a> $View<'a> {
            /// The half-height of the cylinder.
            pub fn half_height(&self) -> Real {
                self.raw.half_height
            }

            /// The base radius of the cylinder.
            pub fn radius(&self) -> Real {
                self.raw.radius
            }
        }
    }
);

impl_ref_methods!(CylinderView);

pub struct CylinderViewMut<'a> {
    pub raw: &'a mut Cylinder,
}

impl_ref_methods!(CylinderViewMut);

impl<'a> CylinderViewMut<'a> {
    /// Set the half-height of the cylinder.
    pub fn set_half_height(&mut self, half_height: Real) {
        self.raw.half_height = half_height;
    }

    /// Set the radius of the basis of the cylinder.
    pub fn set_radius(&mut self, radius: Real) {
        self.raw.radius = radius;
    }
}
