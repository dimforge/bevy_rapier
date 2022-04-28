use crate::math::Real;
use rapier::parry::shape::Ball;

pub struct BallView<'a> {
    pub raw: &'a Ball,
}

macro_rules! impl_ref_methods(
    ($View: ident) => {
        impl<'a> $View<'a> {
            /// The radius of the ball.
            pub fn radius(&self) -> Real {
                self.raw.radius
            }
        }
    }
);

impl_ref_methods!(BallView);

pub struct BallViewMut<'a> {
    pub raw: &'a mut Ball,
}

impl_ref_methods!(BallViewMut);

impl<'a> BallViewMut<'a> {
    /// Set the radius of the ball.
    pub fn set_radius(&mut self, radius: Real) {
        self.raw.radius = radius;
    }
}
