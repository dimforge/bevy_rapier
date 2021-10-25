use bevy::prelude::*;
use std::ops::{Deref,DerefMut};
macro_rules! impl_component_wrapper(
  ($Wrapper:ident, $T: ty) => {
    #[derive(Component)]
    pub struct $Wrapper(pub $T);
    impl Default for $Wrapper{
      fn default()->Self{
        $Wrapper(<$T>::default())
      }
    }
    impl Deref for $Wrapper{
      type Target = $T;
      fn deref(&self)->&Self::Target{
        &self.0
      }
    }
    impl DerefMut for $Wrapper{
      fn deref_mut(&mut self)->&mut Self::Target{
        &mut self.0
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
  ($Wrapper:ident, $T: ty) => {
    #[derive(Component)]
    pub struct $Wrapper(pub $T);
    impl Deref for $Wrapper{
      type Target = $T;
      fn deref(&self)->&Self::Target{
        &self.0
      }
    }
    impl DerefMut for $Wrapper{
      fn deref_mut(&mut self)->&mut Self::Target{
        &mut self.0
      }
    }
  }
);
use rapier2d::{geometry,dynamics};
impl_component_wrapper_nd!(RigidBodyHandle,dynamics::RigidBodyHandle);
impl_component_wrapper_hash!(RigidBodyHandle);
impl_component_wrapper_nd!(RigidBodyType,dynamics::RigidBodyType);
impl_component_wrapper!(RigidBodyChanges,dynamics::RigidBodyChanges);
impl_component_wrapper!(RigidBodyPosition,dynamics::RigidBodyPosition);
impl_component_wrapper!(RigidBodyMassProps,dynamics::RigidBodyMassProps);
impl_component_wrapper!(RigidBodyVelocity,dynamics::RigidBodyVelocity);
impl_component_wrapper!(RigidBodyDamping,dynamics::RigidBodyDamping);
impl_component_wrapper!(RigidBodyForces,dynamics::RigidBodyForces);
impl_component_wrapper!(RigidBodyCcd,dynamics::RigidBodyCcd);
impl_component_wrapper!(RigidBodyIds,dynamics::RigidBodyIds);
impl_component_wrapper_hash!(RigidBodyIds);
impl_component_wrapper!(RigidBodyDominance,dynamics::RigidBodyDominance);
impl_component_wrapper_hash!(RigidBodyDominance);
impl_component_wrapper!(RigidBodyActivation,dynamics::RigidBodyActivation);
impl_component_wrapper!(RigidBodyColliders,dynamics::RigidBodyColliders);
impl_component_wrapper_nd!(ColliderHandle,geometry::ColliderHandle);
impl_component_wrapper_hash!(ColliderHandle);
impl_component_wrapper!(ColliderChanges,geometry::ColliderChanges);
impl_component_wrapper_nd!(ColliderType,geometry::ColliderType);
impl_component_wrapper!(ColliderBroadPhaseData,geometry::ColliderBroadPhaseData);
impl_component_wrapper_hash!(ColliderBroadPhaseData);
impl_component_wrapper!(ColliderMassProps,geometry::ColliderMassProps);
impl_component_wrapper_nd!(ColliderParent,geometry::ColliderParent);
impl_component_wrapper!(ColliderPosition,geometry::ColliderPosition);
impl_component_wrapper!(ColliderMaterial,geometry::ColliderMaterial);
impl_component_wrapper!(ColliderFlags,geometry::ColliderFlags);
impl_component_wrapper_hash!(ColliderFlags);
impl_component_wrapper_nd!(ColliderShape,geometry::ColliderShape);
