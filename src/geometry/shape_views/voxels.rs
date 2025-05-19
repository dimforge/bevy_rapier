use rapier::prelude::{Aabb, VoxelData, VoxelState, Voxels};

use crate::math::{IVect, Vect};

#[cfg(feature = "dim2")]
use bevy::math::bounding::Aabb2d as BevyAabb;
#[cfg(feature = "dim3")]
use bevy::math::bounding::Aabb3d as BevyAabb;

fn aabb_na_from_bevy(aabb: &BevyAabb) -> Aabb {
    rapier::parry::bounding_volume::Aabb::new(aabb.min.into(), aabb.max.into())
}

fn aabb_bevy_from_na(aabb: &Aabb) -> BevyAabb {
    BevyAabb {
        min: aabb.mins.into(),
        max: aabb.maxs.into(),
    }
}

/// Read-only access to the properties of a [`Voxels`] shape.
#[derive(Copy, Clone)]
pub struct VoxelsView<'a> {
    /// The raw shape from Rapier.
    pub raw: &'a Voxels,
}

// TODO: implement those on VoxelsViewMut too.
macro_rules! impl_ref_methods(
    ($View: ident) => {
        impl<'a> $View<'a> {
            /// Shortcut to [`Voxels::total_memory_size`].
            pub fn total_memory_size(&self) -> usize {
                self.raw.total_memory_size()
            }

            /// Shortcut to [`Voxels::heap_memory_size`].
            pub fn heap_memory_size(&self) -> usize {
                self.raw.heap_memory_size()
            }

            /// Shortcut to [`Voxels::extents`].
            pub fn extents(&self) -> Vect {
                self.raw.extents().into()
            }

            /// Shortcut to [`Voxels::domain_center`].
            pub fn domain_center(&self) -> Vect {
                self.raw.domain_center().into()
            }

            /// Shortcut to [`Voxels::is_voxel_in_bounds`].
            pub fn is_voxel_in_bounds(&self, key: IVect) -> bool {
                self.raw.is_voxel_in_bounds(key.into())
            }

            /// Shortcut to [`Voxels::voxel_aabb`].
            pub fn voxel_aabb(&self, key: IVect) -> BevyAabb {
                aabb_bevy_from_na(&self.raw.voxel_aabb(key.into()))
            }

            /// Shortcut to [`Voxels::voxel_state`].
            pub fn voxel_state(&self, key: IVect) -> VoxelState {
                self.raw.voxel_state(key.into())
            }

            /// Shortcut to [`Voxels::voxel_at_point_unchecked`].
            pub fn voxel_at_point_unchecked(&self, point: Vect) -> IVect {
                self.raw.voxel_at_point_unchecked(point.into()).into()
            }

            /// Shortcut to [`Voxels::voxel_at_point`].
            pub fn voxel_at_point(&self, pt: Vect) -> Option<IVect> {
                self.raw.voxel_at_point(pt.into()).map(IVect::from)
            }

            /// Shortcut to [`Voxels::clamp_voxel`].
            pub fn clamp_voxel(&self, key: IVect) -> IVect {
                self.raw.clamp_voxel(key.into()).into()
            }

            /// Shortcut to [`Voxels::voxel_range_intersecting_local_aabb`].
            pub fn voxel_range_intersecting_local_aabb(&self, aabb: &BevyAabb) -> [IVect; 2] {
                let res = self
                    .raw
                    .voxel_range_intersecting_local_aabb(&aabb_na_from_bevy(aabb));
                [res[0].into(), res[1].into()]
            }

            /// Shortcut to [`Voxels::voxel_range_aabb`].
            pub fn voxel_range_aabb(&self, mins: IVect, maxs: IVect) -> BevyAabb {
                aabb_bevy_from_na(&self.raw.voxel_range_aabb(mins.into(), maxs.into()))
            }

            /// Shortcut to [`Voxels::align_aabb_to_grid`].
            pub fn align_aabb_to_grid(&self, aabb: &BevyAabb) -> BevyAabb {
                aabb_bevy_from_na(&self.raw
                    .align_aabb_to_grid(&aabb_na_from_bevy(aabb)))
            }

            /// Shortcut to [`Voxels::voxels_intersecting_local_aabb`].
            pub fn voxels_intersecting_local_aabb(
                &self,
                aabb: &BevyAabb,
            ) -> impl Iterator<Item = VoxelData> + '_ {
                self.raw
                    .voxels_intersecting_local_aabb(&aabb_na_from_bevy(aabb))
            }

            /// Shortcut to [`Voxels::voxels`].
            pub fn voxels(&self) -> impl Iterator<Item = VoxelData> + '_ {
                self.raw.voxels()
            }

            /// Shortcut to [`Voxels::split_with_box`].
            #[cfg(feature = "dim2")]
            pub fn split_with_box(&self, aabb: &BevyAabb) -> (Option<Voxels>, Option<Voxels>) {
                self.raw.split_with_box(&aabb_na_from_bevy(aabb))
            }

            /// Shortcut to [`Voxels::voxels_in_range`].
            pub fn voxels_in_range(
                &self,
                mins: IVect,
                maxs: IVect,
            ) -> impl Iterator<Item = VoxelData> + '_ {
                self.raw.voxels_in_range(mins.into(), maxs.into())
            }
        }
    }
);

impl_ref_methods!(VoxelsView);

/// Read-write access to the properties of a [`Voxels`].
pub struct VoxelsViewMut<'a> {
    /// The raw shape from Rapier.
    pub raw: &'a mut Voxels,
}

impl_ref_methods!(VoxelsViewMut);

impl<'a> VoxelsViewMut<'a> {
    /// Shortcut to [`Voxels::try_set_voxel`].
    pub fn try_set_voxel(&mut self, key: IVect, is_filled: bool) -> Option<VoxelState> {
        self.raw.try_set_voxel(key.into(), is_filled)
    }

    /// Shortcut to [`Voxels::set_voxel`].
    pub fn set_voxel(&mut self, key: IVect, is_filled: bool) -> Option<VoxelState> {
        self.raw.set_voxel(key.into(), is_filled)
    }

    /// Shortcut to [`Voxels::resize_domain`].
    pub fn resize_domain(&mut self, domain_mins: IVect, domain_maxs: IVect) {
        self.raw
            .resize_domain(domain_mins.into(), domain_maxs.into());
    }
}
