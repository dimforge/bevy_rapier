use bevy::prelude::*;
use std::ops::{Deref, DerefMut};
macro_rules! impl_component_wrapper(
    ($Wrapper:ident, $Inner: ty) => {
        #[derive(Component)]
        pub struct $Wrapper(pub $Inner);

        impl Default for $Wrapper {
            fn default() -> Self{
                $Wrapper(<$Inner>::default())
            }
        }

        impl Deref for $Wrapper {
            type Target = $Inner;
            fn deref(&self) -> &Self::Target{
                &self.0
            }
        }

        impl DerefMut for $Wrapper {
            fn deref_mut(&mut self) -> &mut Self::Target{
                &mut self.0
            }
        }

        impl<T> From<T> for $Wrapper
        where
            $Inner: From<T>,
        {
            fn from(t: T) -> Self {
                $Wrapper(t.into())
            }
        }
    }
);

use std::hash::{Hash, Hasher};
macro_rules! impl_component_wrapper_hash(
  ($Wrapper:ident) => {
        impl Hash for $Wrapper{
              fn hash<H: Hasher>(&self, state: &mut H) {
                    self.0.hash(state);
              }
        }

        impl PartialEq for $Wrapper{
              fn eq(&self,other:&Self) -> bool{
                    self.0 == other.0
              }
        }

        impl Eq for $Wrapper {}
  }
);

macro_rules! impl_component_wrapper_nd(
    ($Wrapper:ident, $Inner: ty) => {
        #[derive(Component)]
        pub struct $Wrapper(pub $Inner);

        impl Deref for $Wrapper {
            type Target = $Inner;
            fn deref(&self)->&Self::Target{
               &self.0
            }
        }

        impl DerefMut for $Wrapper {
            fn deref_mut(&mut self)->&mut Self::Target{
                &mut self.0
            }
        }

        impl<T> From<T> for $Wrapper
        where
            $Inner: From<T>,
        {
            fn from(t: T) -> Self {
                $Wrapper(t.into())
            }
        }
    }
);

use rapier::{dynamics, geometry};
impl_component_wrapper_nd!(RigidBodyHandleComponent, dynamics::RigidBodyHandle);
impl_component_wrapper_hash!(RigidBodyHandleComponent);
impl_component_wrapper_nd!(RigidBodyTypeComponent, dynamics::RigidBodyType);
impl_component_wrapper!(RigidBodyChangesComponent, dynamics::RigidBodyChanges);
impl_component_wrapper!(RigidBodyPositionComponent, dynamics::RigidBodyPosition);
impl_component_wrapper!(RigidBodyMassPropsComponent, dynamics::RigidBodyMassProps);
impl_component_wrapper!(RigidBodyVelocityComponent, dynamics::RigidBodyVelocity);
impl_component_wrapper!(RigidBodyDampingComponent, dynamics::RigidBodyDamping);
impl_component_wrapper!(RigidBodyForcesComponent, dynamics::RigidBodyForces);
impl_component_wrapper!(RigidBodyCcdComponent, dynamics::RigidBodyCcd);
impl_component_wrapper!(RigidBodyIdsComponent, dynamics::RigidBodyIds);
impl_component_wrapper_hash!(RigidBodyIdsComponent);
impl_component_wrapper!(RigidBodyDominanceComponent, dynamics::RigidBodyDominance);
impl_component_wrapper_hash!(RigidBodyDominanceComponent);
impl_component_wrapper!(RigidBodyActivationComponent, dynamics::RigidBodyActivation);
impl_component_wrapper!(RigidBodyCollidersComponent, dynamics::RigidBodyColliders);
impl_component_wrapper_nd!(ColliderHandleComponent, geometry::ColliderHandle);
impl_component_wrapper_hash!(ColliderHandleComponent);
impl_component_wrapper!(ColliderChangesComponent, geometry::ColliderChanges);
impl_component_wrapper_nd!(ColliderTypeComponent, geometry::ColliderType);
impl_component_wrapper!(
    ColliderBroadPhaseDataComponent,
    geometry::ColliderBroadPhaseData
);
impl_component_wrapper_hash!(ColliderBroadPhaseDataComponent);
impl_component_wrapper!(ColliderMassPropsComponent, geometry::ColliderMassProps);
impl_component_wrapper_nd!(ColliderParentComponent, geometry::ColliderParent);
impl_component_wrapper!(ColliderPositionComponent, geometry::ColliderPosition);
impl_component_wrapper!(ColliderMaterialComponent, geometry::ColliderMaterial);
impl_component_wrapper!(ColliderFlagsComponent, geometry::ColliderFlags);
impl_component_wrapper_hash!(ColliderFlagsComponent);
impl_component_wrapper_nd!(ColliderShapeComponent, geometry::ColliderShape);
