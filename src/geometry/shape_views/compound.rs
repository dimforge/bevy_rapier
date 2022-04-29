use super::ColliderView;
use crate::math::{Rot, Vect};
use rapier::parry::shape::Compound;

/// Read-only access to the properties of a compound shape.
pub struct CompoundView<'a> {
    /// The raw shape from Rapier.
    pub raw: &'a Compound,
}

impl<'a> CompoundView<'a> {
    /// The shapes of this compound shape.
    #[inline]
    pub fn shapes(&self) -> impl ExactSizeIterator<Item = (Vect, Rot, ColliderView)> {
        self.raw.shapes().iter().map(|(pos, shape)| {
            let (tra, rot) = (*pos).into();
            (tra, rot, shape.as_typed_shape().into())
        })
    }
}
