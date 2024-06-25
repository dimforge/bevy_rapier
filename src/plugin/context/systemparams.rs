use bevy::ecs::system::SystemParam;
use bevy::prelude::*;
use rapier::dynamics::RigidBodyHandle;
use std::collections::HashMap;
use std::ops::{Deref, DerefMut};

use super::{DefaultRapierContext, RapierContext, RapierContextEntityLink};

/// Utility [`SystemParam`] to easily access the default world [`RapierContext`] immutably
#[derive(SystemParam)]
pub struct DefaultRapierContextAccess<'w, 's> {
    rapier_context: Query<'w, 's, &'static RapierContext, With<DefaultRapierContext>>,
}

impl<'w, 's> DefaultRapierContextAccess<'w, 's> {
    /// Use this method if you only have one world.
    pub fn single<'a>(&'_ self) -> &RapierContext {
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
    rapier_context: Query<'w, 's, &'static RapierContext, With<DefaultRapierContext>>,
}

impl<'w, 's> RapierContextAccess<'w, 's> {
    /// Use this method if you only have one world.
    pub fn single<'a>(&'_ self) -> &RapierContext {
        self.rapier_context.single()
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
    rapier_context: Query<'w, 's, &'static mut RapierContext>,
    rapier_context_link: Query<'w, 's, &'static mut RapierContextEntityLink>,
}

impl<'w, 's> RapierContextAccessMut<'w, 's> {
    pub fn context(&mut self, entity: Entity) -> &'_ mut RapierContext {
        let context_link = self
            .rapier_context_link
            .get(entity)
            .expect("entity2body called on an entity without RapierContextEntityLink.");
        self.rapier_context
            .get_mut(context_link.0)
            .expect("RapierContextEntityLink refers to an entity without RapierContext.")
            .into_inner()
    }
}

pub fn try_retrieve_context<'a>(
    link: Option<&RapierContextEntityLink>,
    context: &Query<(Entity, &mut RapierContext)>,
) -> Result<Entity, Entity> {
    link.map_or_else(
        || {
            let context_entity = context.iter().next().unwrap().0;
            RapierContextEntityLink(context_entity);
            Err(context_entity)
        },
        |link| Ok(link.0),
    )
}
