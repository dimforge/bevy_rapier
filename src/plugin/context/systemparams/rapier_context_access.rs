use bevy::ecs::system::SystemParam;
use bevy::prelude::*;
use std::ops::{Deref, DerefMut};

use super::super::{DefaultRapierContext, RapierContext, RapierContextEntityLink};
/// Utility [`SystemParam`] to easily access the single default [`RapierContext`] immutably.
///
/// SAFETY: Dereferencing this struct will panic if its underlying query fails.
/// See [`RapierContextAccess`] for a safer alternative.
#[derive(SystemParam)]
pub struct DefaultRapierContextAccess<'w, 's, T: Component = DefaultRapierContext> {
    rapier_context: Query<'w, 's, &'static RapierContext, With<T>>,
}

impl<'w, 's, T: Component> DefaultRapierContextAccess<'w, 's, T> {
    /// Use this method if you only have one [`RapierContext`].
    ///
    /// SAFETY: This method will panic if its underlying query fails.
    /// See [`RapierContextAccess`] for a safe alternative.
    pub fn single(&'_ self) -> &RapierContext {
        self.rapier_context.single()
    }
}

impl<'w, 's> Deref for DefaultRapierContextAccess<'w, 's> {
    type Target = RapierContext;

    /// Use this method if you only have one [`RapierContext`].
    ///
    /// SAFETY: This method will panic if its underlying query fails.
    /// See [`RapierContextAccess`] for a safe alternative.
    fn deref(&self) -> &Self::Target {
        self.rapier_context.single()
    }
}

/// Utility [`SystemParam`] to easily access the single default [`RapierContext`] mutably.
///
/// SAFETY: Dereferencing this struct will panic if its underlying query fails.
/// See [`RapierContextAccess`] for a safer alternative.
#[derive(SystemParam)]
pub struct DefaultRapierContextAccessMut<'w, 's, T: Component = DefaultRapierContext> {
    rapier_context: Query<'w, 's, &'static mut RapierContext, With<T>>,
}

impl<'w, 's, T: Component> Deref for DefaultRapierContextAccessMut<'w, 's, T> {
    type Target = RapierContext;

    /// Use this method if you only have one [`RapierContext`].
    ///
    /// SAFETY: This method will panic if its underlying query fails.
    /// See [`RapierContextAccess`] for a safe alternative.
    fn deref(&self) -> &Self::Target {
        self.rapier_context.single()
    }
}

impl<'w, 's> DerefMut for DefaultRapierContextAccessMut<'w, 's> {
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
        self.try_context(link)
            .expect("RapierContextEntityLink.0 refers to an entity without RapierContext.")
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
pub struct RapierContextAccessMut<'w, 's> {
    /// Query used to retrieve a [`RapierContext`].
    /// It's helpful to iterate over every rapier contexts,
    /// or get a handle over a specific context, for example through:
    /// - a marker component such as [`DefaultRapierContext`]
    /// - a [`RapierContextEntityLink`]. See [context](RapierContextAccess::context)
    pub rapier_context: Query<'w, 's, &'static mut RapierContext>,
}

impl<'w, 's> RapierContextAccessMut<'w, 's> {
    /// Retrieves the rapier context responsible for the entity owning the given [`RapierContextEntityLink`].
    ///
    /// SAFETY: This method will panic if its underlying query fails.
    /// See [`Self::try_context`] for a safe alternative.
    pub fn context(&mut self, link: &RapierContextEntityLink) -> Mut<RapierContext> {
        self.try_context(link)
            .expect("RapierContextEntityLink.0 refers to an entity without RapierContext.")
    }

    /// Retrieves the rapier context responsible for the entity owning the given [`RapierContextEntityLink`].
    pub fn try_context(&mut self, link: &RapierContextEntityLink) -> Option<Mut<RapierContext>> {
        self.rapier_context.get_mut(link.0).ok()
    }

    /// Retrieves the rapier context component on this [`Entity`].
    ///
    /// Calling this method on a rapier managed entity (rigid body, collider, joints...) will fail.
    /// Given entity should have a [`RapierContext`].
    ///
    /// SAFETY: This method will panic if its underlying query fails.
    pub(crate) fn context_from_entity(
        &mut self,
        rapier_context_entity: Entity,
    ) -> Mut<RapierContext> {
        self.try_context_from_entity(rapier_context_entity)
            .unwrap_or_else(|| panic!("entity {rapier_context_entity} has no RapierContext."))
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

/// Gets the default RapierContext.
///
/// SAFETY: Panics if no entity with [`DefaultRapierContext`] exist.
pub fn get_single_context(
    default_context_access: &Query<Entity, With<DefaultRapierContext>>,
) -> Entity {
    default_context_access.get_single().expect(
        "No entity with `DefaultRapierContext` found.\
                        Please add a default `RapierContext` or a `RapierContextEntityLink`\
                        on the new rapier-managed entity.",
    )
}
