use bevy::ecs::system::SystemParam;
use bevy::prelude::*;
use std::ops::{Deref, DerefMut};

use super::{DefaultRapierContext, RapierContext, RapierContextEntityLink};

/// Utility [`SystemParam`] to easily access the default world [`RapierContext`] immutably
#[derive(SystemParam)]
pub struct DefaultRapierContextAccess<'w, 's> {
    rapier_context: Query<'w, 's, &'static RapierContext, With<DefaultRapierContext>>,
}

impl<'w, 's> DefaultRapierContextAccess<'w, 's> {
    /// Use this method if you only have one world.
    pub fn single(&'_ self) -> &RapierContext {
        self.rapier_context.single()
    }
}

impl<'w, 's> Deref for DefaultRapierContextAccess<'w, 's> {
    type Target = RapierContext;

    fn deref(&self) -> &Self::Target {
        self.rapier_context.single()
    }
}

/// Utility [`SystemParam`] to easily access the default world [`RapierContext`] mutably
#[derive(SystemParam)]
pub struct DefaultRapierContextAccessMut<'w, 's> {
    rapier_context: Query<'w, 's, &'static mut RapierContext, With<DefaultRapierContext>>,
}

impl<'w, 's> Deref for DefaultRapierContextAccessMut<'w, 's> {
    type Target = RapierContext;

    fn deref(&self) -> &Self::Target {
        self.rapier_context.single()
    }
}

impl<'w, 's> DerefMut for DefaultRapierContextAccessMut<'w, 's> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        // TODO: should we cache the result ?
        self.rapier_context.single_mut().into_inner()
    }
}

/// Utility [`SystemParam`] to easily access any [`RapierContext`] immutably
#[derive(SystemParam)]
pub struct RapierContextAccess<'w, 's> {
    rapier_context: Query<'w, 's, &'static RapierContext>,
    rapier_context_link: Query<'w, 's, &'static RapierContextEntityLink>,
}

impl<'w, 's> RapierContextAccess<'w, 's> {
    pub fn link(&self, entity: Entity) -> &RapierContextEntityLink {
        self.rapier_context_link
            .get(entity)
            .expect("RapierContextAccess.link called on an entity without RapierContextEntityLink.")
    }
    pub fn follow_link(&self, link: RapierContextEntityLink) -> &'_ RapierContext {
        self.rapier_context
            .get(link.0)
            .expect("RapierContextEntityLink.0 refers to an entity without RapierContext.")
    }

    pub fn context(&self, entity: Entity) -> &'_ RapierContext {
        let context_link = self.link(entity);
        self.follow_link(*context_link)
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
    pub rapier_context: Query<'w, 's, &'static mut RapierContext>,
    pub rapier_context_link: Query<'w, 's, &'static RapierContextEntityLink>,
}

impl<'w, 's> RapierContextAccessMut<'w, 's> {
    pub fn link(&self, entity: Entity) -> &RapierContextEntityLink {
        self.rapier_context_link.get(entity).expect(
            "RapierContextAccessMut.link called on an entity without RapierContextEntityLink.",
        )
    }
    pub fn follow_link(&mut self, link: RapierContextEntityLink) -> &'_ mut RapierContext {
        self.rapier_context
            .get_mut(link.0)
            .expect("RapierContextEntityLink.0 refers to an entity without RapierContext.")
            .into_inner()
    }

    pub fn context(&mut self, entity: Entity) -> &'_ mut RapierContext {
        let context_link = self.link(entity);
        self.follow_link(*context_link)
    }
}

pub fn try_retrieve_context<'a>(
    link: Option<&RapierContextEntityLink>,
    context: &Query<(Entity, &mut RapierContext)>,
) -> Result<Entity, Entity> {
    link.map_or_else(
        || {
            let context_entity = context.iter().next().unwrap().0;
            Err(context_entity)
        },
        |link| Ok(link.0),
    )
}
