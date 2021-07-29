pub use self::collider_component_set::*;
pub use self::components::*;
pub use self::plugins::*;
pub use self::resources::*;
pub use self::rigid_body_component_set::*;
pub use self::systems::*;
pub use self::mesh_collider::*;

use crate::rapier::data::{ComponentSet, ComponentSetMut, ComponentSetOption, Index};
use crate::rapier::prelude::*;
use bevy::prelude::{Entity, Query, QuerySet};

pub trait IntoHandle<H> {
    fn handle(self) -> H;
}

pub trait IntoEntity {
    fn entity(self) -> Entity;
}

impl IntoHandle<Index> for Entity {
    #[inline]
    fn handle(self) -> Index {
        Index::from_raw_parts(self.id(), self.generation())
    }
}

impl IntoEntity for Index {
    #[inline]
    fn entity(self) -> Entity {
        let (id, gen) = self.into_raw_parts();
        let bits = u64::from(gen) << 32 | u64::from(id);
        Entity::from_bits(bits)
    }
}

impl IntoHandle<JointHandle> for Entity {
    #[inline]
    fn handle(self) -> JointHandle {
        JointHandle::from_raw_parts(self.id(), self.generation())
    }
}

impl IntoEntity for JointHandle {
    fn entity(self) -> Entity {
        self.0.entity()
    }
}

pub trait BundleBuilder {
    type Bundle;
    fn bundle(&self) -> Self::Bundle;
}

macro_rules! impl_component_set_mut(
    ($ComponentsSet: ident, $T: ty, |$data: ident| $data_expr: expr) => {
        impl<'a, 'b, 'c> ComponentSetOption<$T> for $ComponentsSet<'a, 'b, 'c> {
            #[inline(always)]
            fn get(&self, handle: Index) -> Option<&$T> {
                self.0.q0().get_component(handle.entity()).ok()
            }
        }

        impl<'a, 'b, 'c> ComponentSet<$T> for $ComponentsSet<'a, 'b, 'c> {
            #[inline(always)]
            fn size_hint(&self) -> usize {
                0
            }

            #[inline(always)]
            fn for_each(&self, mut f: impl FnMut(Index, &$T)) {
                self.0.q0().for_each(|$data| f($data.0.handle(), $data_expr))
            }
        }

        impl<'a, 'b, 'c> ComponentSetMut<$T> for $ComponentsSet<'a, 'b, 'c> {
            #[inline(always)]
            fn set_internal(&mut self, handle: Index, val: $T) {
                let _ = self.0.q1_mut().get_component_mut(handle.entity()).map(|mut data| *data = val);
            }

            #[inline(always)]
            fn map_mut_internal<Result>(
                &mut self,
                handle: Index,
                f: impl FnOnce(&mut $T) -> Result,
            ) -> Option<Result> {
                self.0.q1_mut()
                    .get_component_mut(handle.entity())
                    .map(|mut data| f(&mut data))
                    .ok()
            }
        }
    }
);

macro_rules! impl_component_set_wo_query_set(
    ($ComponentsSet: ident, $T: ty, |$data: ident| $data_expr: expr) => {
        impl<'a, 'b, 'c> ComponentSetOption<$T> for $ComponentsSet<'a, 'b, 'c> {
            #[inline(always)]
            fn get(&self, handle: Index) -> Option<&$T> {
                self.0.get_component(handle.entity()).ok()
            }
        }

        impl<'a, 'b, 'c> ComponentSet<$T> for $ComponentsSet<'a, 'b, 'c> {
            #[inline(always)]
            fn size_hint(&self) -> usize {
                0
            }

            #[inline(always)]
            fn for_each(&self, mut f: impl FnMut(Index, &$T)) {
                self.0.for_each(|$data| f($data.0.handle(), $data_expr))
            }
        }
    }
);

macro_rules! impl_component_set(
    ($ComponentsSet: ident, $T: ty, |$data: ident| $data_expr: expr) => {
        impl<'a, 'b, 'c> ComponentSetOption<$T> for $ComponentsSet<'a, 'b, 'c> {
            #[inline(always)]
            fn get(&self, handle: Index) -> Option<&$T> {
                self.0.q0().get_component(handle.entity()).ok()
            }
        }

        impl<'a, 'b, 'c> ComponentSet<$T> for $ComponentsSet<'a, 'b, 'c> {
            #[inline(always)]
            fn size_hint(&self) -> usize {
                0
            }

            #[inline(always)]
            fn for_each(&self, mut f: impl FnMut(Index, &$T)) {
                self.0.q0().for_each(|$data| f($data.0.handle(), $data_expr))
            }
        }
    }
);

macro_rules! impl_component_set_option(
    ($ComponentsSet: ident, $T: ty) => {
        impl<'a, 'b, 'c> ComponentSetOption<$T> for $ComponentsSet<'a, 'b, 'c> {
            #[inline(always)]
            fn get(&self, handle: Index) -> Option<&$T> {
                self.0.q0().get_component(handle.entity()).ok()
            }
        }
    }
);

pub type ComponentSetQueryMut<'a, 'b, 'c, T> =
    QuerySet<(Query<'a, (Entity, &'b T)>, Query<'a, (Entity, &'c mut T)>)>;

pub struct QueryComponentSetMut<'a, 'b, 'c, T: 'static + Send + Sync>(
    ComponentSetQueryMut<'a, 'b, 'c, T>,
);

impl<'a, 'b, 'c, T: 'static + Send + Sync> ComponentSetOption<T>
    for QueryComponentSetMut<'a, 'b, 'c, T>
{
    #[inline(always)]
    fn get(&self, handle: Index) -> Option<&T> {
        self.0.q0().get_component(handle.entity()).ok()
    }
}

impl<'a, 'b, 'c, T: 'static + Send + Sync> ComponentSet<T> for QueryComponentSetMut<'a, 'b, 'c, T> {
    #[inline(always)]
    fn size_hint(&self) -> usize {
        0
    }

    #[inline(always)]
    fn for_each(&self, mut f: impl FnMut(Index, &T)) {
        self.0.q0().for_each(|data| f(data.0.handle(), &data.1))
    }
}

impl<'a, 'b, 'c, T: 'static + Send + Sync> ComponentSetMut<T>
    for QueryComponentSetMut<'a, 'b, 'c, T>
{
    #[inline(always)]
    fn set_internal(&mut self, handle: Index, val: T) {
        let _ = self
            .0
            .q1_mut()
            .get_mut(handle.entity())
            .map(|mut data| *data.1 = val);
    }

    #[inline(always)]
    fn map_mut_internal<Result>(
        &mut self,
        handle: Index,
        f: impl FnOnce(&mut T) -> Result,
    ) -> Option<Result> {
        self.0
            .q1_mut()
            .get_component_mut(handle.entity())
            .map(|mut data| f(&mut data))
            .ok()
    }
}

mod collider_component_set;
mod components;
mod plugins;
mod resources;
mod rigid_body_component_set;
mod systems;
mod mesh_collider;
