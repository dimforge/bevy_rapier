use crate::{
    math::{Rot, Vect},
    plugin::context::RapierRigidBodySet,
};
use bevy::ecs::system::SystemParam;
use bevy::prelude::*;
use rapier::prelude::Real;
use std::ops::{Deref, DerefMut};

pub const RAPIER_CONTEXT_EXPECT_ERROR: &str =
    "RapierContextEntityLink.0 refers to an entity without RapierContext.";

use crate::{
    plugin::{context::RapierQueryPipeline, RapierContextColliders, RapierContextJoints},
    prelude::QueryFilter,
};

use super::super::{DefaultRapierContext, RapierContext, RapierContextEntityLink};

/// Utility [`SystemParam`] to easily access the single default [`RapierContext`] immutably.
///
/// SAFETY: Dereferencing this struct will panic if its underlying query fails.
/// See [`RapierContextAccess`] for a safer alternative.
#[derive(SystemParam)]
pub struct ReadDefaultRapierContext<'w, 's, T: Component = DefaultRapierContext> {
    rapier_context: Query<
        'w,
        's,
        (
            &'static RapierContext,
            &'static RapierContextColliders,
            &'static RapierContextJoints,
            &'static RapierQueryPipeline,
            &'static RapierRigidBodySet,
        ),
        With<T>,
    >,
}

impl<'w, 's, T: Component> ReadDefaultRapierContext<'w, 's, T> {
    /// Use this method if you only have one [`RapierContext`].
    ///
    /// SAFETY: This method will panic if its underlying query fails.
    /// See [`RapierContextAccess`] for a safe alternative.
    pub fn single(&'_ self) -> RapierContextWhole {
        let (context, colliders, joints, query_pipeline, rigidbody_set) =
            self.rapier_context.single();
        RapierContextWhole {
            context,
            colliders,
            joints,
            query_pipeline,
            rigidbody_set: rigidbody_set,
        }
    }
}

/// A helper struct to avoid passing too many parameters to most rapier functions.
/// This helps with reducing boilerplate, at the (small) price of maybe getting too much information from the ECS.
pub struct RapierContextWhole<'a> {
    pub context: &'a RapierContext,
    pub colliders: &'a RapierContextColliders,
    pub joints: &'a RapierContextJoints,
    pub query_pipeline: &'a RapierQueryPipeline,
    pub rigidbody_set: &'a RapierRigidBodySet,
}

impl<'a> RapierContextWhole<'a> {
    /// Shortcut to [`RapierContext::impulse_revolute_joint_angle`]
    pub fn impulse_revolute_joint_angle(&self, entity: Entity) -> Option<f32> {
        self.rigidbody_set
            .impulse_revolute_joint_angle(self.joints, entity)
    }
    /// Shortcut to [`RapierQueryPipeline::cast_ray`]
    pub fn cast_ray(
        &self,
        ray_origin: Vect,
        ray_dir: Vect,
        max_toi: Real,
        solid: bool,
        filter: QueryFilter,
    ) -> Option<(Entity, Real)> {
        self.query_pipeline.cast_ray(
            &self.colliders,
            &self.rigidbody_set,
            ray_origin,
            ray_dir,
            max_toi,
            solid,
            filter,
        )
    }
}

/// Utility [`SystemParam`] to easily access the single default [`RapierContextColliders`] immutably.
///
/// SAFETY: Dereferencing this struct will panic if its underlying query fails.
/// See [`RapierContextAccess`] for a safer alternative.
#[derive(SystemParam)]
pub struct ReadDefaultRapierContextColliders<'w, 's, T: Component = DefaultRapierContext> {
    rapier_context_colliders: Query<'w, 's, &'static RapierContextColliders, With<T>>,
}

impl<'w, 's, T: Component> ReadDefaultRapierContextColliders<'w, 's, T> {
    /// Use this method if you only have one [`RapierContext`].
    ///
    /// SAFETY: This method will panic if its underlying query fails.
    /// See [`RapierContextAccess`] for a safe alternative.
    pub fn single(&'_ self) -> &RapierContextColliders {
        self.rapier_context_colliders.single()
    }
}

impl<'w, 's> Deref for ReadDefaultRapierContextColliders<'w, 's> {
    type Target = RapierContextColliders;

    /// Use this method if you only have one [`RapierContext`].
    ///
    /// SAFETY: This method will panic if its underlying query fails.
    /// See [`RapierContextAccess`] for a safe alternative.
    fn deref(&self) -> &Self::Target {
        self.rapier_context_colliders.single()
    }
}

/// Utility [`SystemParam`] to easily access the single default [`RapierContext`] mutably.
///
/// SAFETY: Dereferencing this struct will panic if its underlying query fails.
/// See [`RapierContextAccess`] for a safer alternative.
#[derive(SystemParam)]
pub struct WriteDefaultRapierContext<'w, 's, T: Component = DefaultRapierContext> {
    rapier_context: Query<'w, 's, &'static mut RapierContext, With<T>>,
}

impl<'w, 's, T: Component> Deref for WriteDefaultRapierContext<'w, 's, T> {
    type Target = RapierContext;

    /// Use this method if you only have one [`RapierContext`].
    ///
    /// SAFETY: This method will panic if its underlying query fails.
    /// See [`RapierContextAccess`] for a safe alternative.
    fn deref(&self) -> &Self::Target {
        self.rapier_context.single()
    }
}

impl<'w, 's> DerefMut for WriteDefaultRapierContext<'w, 's> {
    /// Use this method if you only have one [`RapierContext`].
    ///
    /// SAFETY: This method will panic if its underlying query fails.
    /// See [`RapierContextAccess`] for a safe alternative.
    fn deref_mut(&mut self) -> &mut Self::Target {
        // TODO: should we cache the result ?
        self.rapier_context.single_mut().into_inner()
    }
}

/// Utility [`SystemParam`] to easily access any [`RapierContext`] immutably
#[derive(SystemParam)]
pub struct RapierContextAccess<'w, 's> {
    /// Query used to retrieve a [`RapierContext`].
    /// It's helpful to iterate over every rapier contexts,
    /// or get a handle over a specific context, for example through:
    /// - a marker component such as [`DefaultRapierContext`]
    /// - a [`RapierContextEntityLink`]. See [context](RapierContextAccess::context)
    pub rapier_context: Query<'w, 's, &'static RapierContext>,
}

impl<'w, 's> RapierContextAccess<'w, 's> {
    /// Retrieves the rapier context responsible for the entity owning the given [`RapierContextEntityLink`].
    ///
    /// SAFETY: This method will panic if its underlying query fails.
    /// See [`Self::try_context`] for a safe alternative.
    pub fn context(&self, link: &RapierContextEntityLink) -> &'_ RapierContext {
        self.try_context(link).expect(RAPIER_CONTEXT_EXPECT_ERROR)
    }

    /// Retrieves the rapier context responsible for the entity owning the given [`RapierContextEntityLink`].
    pub fn try_context(&self, link: &RapierContextEntityLink) -> Option<&'_ RapierContext> {
        self.rapier_context.get(link.0).ok()
    }
}

impl<'w, 's> Deref for RapierContextAccess<'w, 's> {
    type Target = RapierContext;

    fn deref(&self) -> &Self::Target {
        self.rapier_context.single()
    }
}

/// Utility [`SystemParam`] to easily access any [`RapierContext`] mutably
#[derive(SystemParam)]
pub struct WriteRapierContext<'w, 's> {
    /// Query used to retrieve a [`RapierContext`].
    /// It's helpful to iterate over every rapier contexts,
    /// or get a handle over a specific context, for example through:
    /// - a marker component such as [`DefaultRapierContext`]
    /// - a [`RapierContextEntityLink`]. See [context](RapierContextAccess::context)
    pub rapier_context: Query<'w, 's, &'static mut RapierContext>,
}

impl<'w, 's> WriteRapierContext<'w, 's> {
    /// Retrieves the rapier context responsible for the entity owning the given [`RapierContextEntityLink`].
    ///
    /// SAFETY: This method will panic if its underlying query fails.
    /// See [`Self::try_context`] for a safe alternative.
    pub fn context(&mut self, link: &RapierContextEntityLink) -> Mut<RapierContext> {
        self.try_context(link).expect(RAPIER_CONTEXT_EXPECT_ERROR)
    }

    /// Retrieves the rapier context responsible for the entity owning the given [`RapierContextEntityLink`].
    pub fn try_context(&mut self, link: &RapierContextEntityLink) -> Option<Mut<RapierContext>> {
        self.rapier_context.get_mut(link.0).ok()
    }

    /// Retrieves the rapier context component on this [`Entity`].
    ///
    /// Calling this method on a rapier managed entity (rigid body, collider, joints...) will fail.
    /// Given entity should have a [`RapierContext`].
    pub fn try_context_from_entity(
        &mut self,
        rapier_context_entity: Entity,
    ) -> Option<Mut<RapierContext>> {
        self.rapier_context.get_mut(rapier_context_entity).ok()
    }
}
