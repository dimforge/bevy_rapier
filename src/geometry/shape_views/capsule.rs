use super::SegmentView;
use crate::math::{Real, Rot, Vect};
use rapier::parry::shape::Capsule;

pub struct CapsuleView<'a> {
    pub raw: &'a Capsule,
}

macro_rules! impl_ref_methods(
    ($View: ident) => {
        impl<'a> $View<'a> {
            /// The axis and endpoint of the capsule.
            pub fn segment(&self) -> SegmentView {
                SegmentView {
                    raw: &self.raw.segment,
                }
            }

            /// The radius of the capsule.
            pub fn radius(&self) -> Real {
                self.raw.radius
            }

            /// The height of this capsule.
            pub fn height(&self) -> Real {
                self.raw.height()
            }

            /// The half-height of this capsule.
            pub fn half_height(&self) -> Real {
                self.raw.half_height()
            }

            /// The center of this capsule.
            pub fn center(&self) -> Vect {
                self.raw.center().into()
            }

            /// The transformation such that `t * Y` is collinear with `b - a` and `t * origin` equals
            /// the capsule's center.
            pub fn canonical_transform(&self) -> (Vect, Rot) {
                self.raw.canonical_transform().into()
            }

            /// The rotation `r` such that `r * Y` is collinear with `b - a`.
            #[cfg(feature = "dim2")]
            pub fn rotation_wrt_y(&self) -> Rot {
                self.raw.rotation_wrt_y().angle()
            }

            /// The rotation `r` such that `r * Y` is collinear with `b - a`.
            #[cfg(feature = "dim3")]
            pub fn rotation_wrt_y(&self) -> Rot {
                self.raw.rotation_wrt_y().into()
            }

            /// The transform `t` such that `t * Y` is collinear with `b - a` and such that `t * origin = (b + a) / 2.0`.
            pub fn transform_wrt_y(&self) -> (Vect, Rot) {
                self.raw.transform_wrt_y().into()
            }
        }
    }
);

impl_ref_methods!(CapsuleView);

pub struct CapsuleViewMut<'a> {
    pub raw: &'a mut Capsule,
}

impl_ref_methods!(CapsuleViewMut);

impl<'a> CapsuleViewMut<'a> {
    /// Set the segment of this capsule.
    pub fn set_segment(&mut self, a: Vect, b: Vect) {
        self.raw.segment.a = a.into();
        self.raw.segment.b = b.into();
    }

    /// Set the radius of this capsule.
    pub fn set_radius(&mut self, radius: Real) {
        self.raw.radius = radius;
    }
}
