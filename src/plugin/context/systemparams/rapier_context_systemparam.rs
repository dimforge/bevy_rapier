use crate::math::{Rot, Vect};
use bevy::ecs::{query, system::SystemParam};
use bevy::prelude::*;
use rapier::prelude::Real;

pub(crate) const RAPIER_CONTEXT_EXPECT_ERROR: &str =
    "RapierContextEntityLink.0 refers to an entity missing components from RapierContextSimulation.";

use crate::plugin::context::{
    DefaultRapierContext, RapierContextColliders, RapierContextJoints, RapierContextSimulation,
    RapierQueryPipeline, RapierRigidBodySet,
};

/// Utility [`SystemParam`] to easily access every required components of a [`RapierContext`] immutably.
///
/// This uses the [`DefaultRapierContext`] filter by default, but you can use a custom query filter with the `T` type parameter.
#[derive(SystemParam)]
pub struct ReadRapierContext<'w, 's, T: query::QueryFilter + 'static = With<DefaultRapierContext>> {
    /// The query used to feed components into [`RapierContext`] struct through [`ReadRapierContext::single`].
    pub rapier_context: Query<
        'w,
        's,
        (
            &'static RapierContextSimulation,
            &'static RapierContextColliders,
            &'static RapierContextJoints,
            &'static RapierRigidBodySet,
        ),
        T,
    >,
}

impl<'w, 's, T: query::QueryFilter + 'static> ReadRapierContext<'w, 's, T> {
    /// Returns a single [`RapierContext`] corresponding to the filter (T) of [`ReadRapierContext`].
    ///
    /// If the number of query items is not exactly one, a [`bevy::ecs::query::QuerySingleError`] is returned instead.
    ///
    /// You can also use the underlying query [`ReadRapierContext::rapier_context`] for finer grained queries.
    pub fn single(&self) -> Result<RapierContext<'_>> {
        let (simulation, colliders, joints, rigidbody_set) = self.rapier_context.single()?;
        Ok(RapierContext {
            simulation,
            colliders,
            joints,
            rigidbody_set,
        })
    }
}

/// A helper struct to avoid passing too many parameters to most rapier functions.
/// This helps with reducing boilerplate, at the (small) price of maybe getting too much information from the ECS.
///
/// Note: This is not a component, refer to [`ReadRapierContext`], [`WriteRapierContext`], or [`RapierContextSimulation`]
#[cfg_attr(feature = "serde-serialize", derive(Serialize))]
#[derive(query::QueryData)]
pub struct RapierContext<'a> {
    /// The Rapier context, containing all the state of the physics engine.
    pub simulation: &'a RapierContextSimulation,
    /// The set of colliders part of the simulation.
    pub colliders: &'a RapierContextColliders,
    /// The sets of joints part of the simulation.
    pub joints: &'a RapierContextJoints,
    /// The set of rigid-bodies part of the simulation.
    pub rigidbody_set: &'a RapierRigidBodySet,
}

/// Utility [`SystemParam`] to easily access every required components of a [`RapierContext`] mutably.
///
/// This uses the [`DefaultRapierContext`] filter by default, but you can use a custom query filter with the `T` type parameter.
#[derive(SystemParam)]
pub struct WriteRapierContext<'w, 's, T: query::QueryFilter + 'static = With<DefaultRapierContext>>
{
    /// The query used to feed components into [`RapierContext`] struct through [`ReadRapierContext::single`].
    pub rapier_context: Query<
        'w,
        's,
        (
            &'static mut RapierContextSimulation,
            &'static mut RapierContextColliders,
            &'static mut RapierContextJoints,
            &'static mut RapierRigidBodySet,
        ),
        T,
    >,
}

impl<'w, 's, T: query::QueryFilter + 'static> WriteRapierContext<'w, 's, T> {
    /// Returns a single [`RapierContext`] corresponding to the filter (T) of [`WriteRapierContext`].
    ///
    /// If the number of query items is not exactly one, a [`bevy::ecs::query::QuerySingleError`] is returned instead.
    ///
    /// You can also use the underlying query [`WriteRapierContext::rapier_context`] for finer grained queries.
    pub fn single(&self) -> Result<RapierContext<'_>> {
        let (simulation, colliders, joints, rigidbody_set) = self.rapier_context.single()?;
        Ok(RapierContext {
            simulation,
            colliders,
            joints,
            rigidbody_set,
        })
    }

    /// Returns a single mutable [`RapierContextMut`] corresponding to the filter (T) of [`WriteRapierContext`].
    ///
    /// If the number of query items is not exactly one, a [`bevy::ecs::query::QuerySingleError`] is returned instead.
    ///
    /// You can also use the underlying query [`WriteRapierContext::rapier_context`] for finer grained queries.
    pub fn single_mut(&mut self) -> Result<RapierContextMut<'_>> {
        let (simulation, colliders, joints, rigidbody_set) = self.rapier_context.single_mut()?;
        Ok(RapierContextMut {
            simulation,
            colliders,
            joints,
            rigidbody_set,
        })
    }
}

/// A helper struct to avoid passing too many parameters to most rapier functions.
/// This helps with reducing boilerplate, at the (small) price of maybe getting too much information from the ECS.
///
/// If you need more granular control over mutability of each component, use a regular [`Query`]
pub struct RapierContextMut<'a> {
    /// The Rapier context, containing all the state of the physics engine.
    pub simulation: Mut<'a, RapierContextSimulation>,
    /// The set of colliders part of the simulation.
    pub colliders: Mut<'a, RapierContextColliders>,
    /// The sets of joints part of the simulation.
    pub joints: Mut<'a, RapierContextJoints>,
    /// The set of rigid-bodies part of the simulation.
    pub rigidbody_set: Mut<'a, RapierRigidBodySet>,
}

/// [`RapierRigidBodySet`] functions
mod simulation {
    use crate::control::CharacterCollision;
    use crate::control::MoveShapeOptions;
    use crate::control::MoveShapeOutput;
    use crate::plugin::context::SimulationToRenderTime;
    use crate::plugin::ContactPairView;
    use crate::plugin::TimestepMode;
    use crate::prelude::CollisionEvent;
    use crate::prelude::ContactForceEvent;
    use crate::prelude::RapierRigidBodyHandle;
    use crate::prelude::TransformInterpolation;
    use rapier::prelude::PhysicsHooks;
    use rapier::prelude::QueryPipelineMut;
    use rapier::prelude::Shape;

    use super::*;

    /// [`RapierContextSimulation`] functions for immutable accesses
    impl RapierContext<'_> {
        /// Shortcut to [`RapierContextSimulation::contact_pair`].
        pub fn contact_pair(
            &self,
            collider1: Entity,
            collider2: Entity,
        ) -> Option<ContactPairView<'_>> {
            self.simulation
                .contact_pair(self.colliders, self.rigidbody_set, collider1, collider2)
        }

        /// Shortcut to [`RapierContextSimulation::contact_pairs_with`].
        pub fn contact_pairs_with(
            &self,
            collider: Entity,
        ) -> impl Iterator<Item = ContactPairView<'_>> {
            self.simulation
                .contact_pairs_with(self.colliders, self.rigidbody_set, collider)
        }

        /// Shortcut to [`RapierContextSimulation::intersection_pair`].
        pub fn intersection_pair(&self, collider1: Entity, collider2: Entity) -> Option<bool> {
            self.simulation
                .intersection_pair(self.colliders, collider1, collider2)
        }

        /// Shortcut to [`RapierContextSimulation::intersection_pairs_with`].
        pub fn intersection_pairs_with(
            &self,
            collider: Entity,
        ) -> impl Iterator<Item = (Entity, Entity, bool)> + '_ {
            self.simulation
                .intersection_pairs_with(self.colliders, collider)
        }
    }

    /// [`RapierContextSimulation`] functions for mutable accesses
    impl RapierContextMut<'_> {
        /// Shortcut to [`RapierContextSimulation::step_simulation`].
        #[expect(clippy::too_many_arguments)]
        pub fn step_simulation(
            &mut self,
            gravity: Vect,
            timestep_mode: TimestepMode,
            events: Option<(
                &EventWriter<CollisionEvent>,
                &EventWriter<ContactForceEvent>,
            )>,
            hooks: &dyn PhysicsHooks,
            time: &Time,
            sim_to_render_time: &mut SimulationToRenderTime,
            interpolation_query: Option<
                &mut Query<(&RapierRigidBodyHandle, &mut TransformInterpolation)>,
            >,
        ) {
            self.simulation.step_simulation(
                &mut self.colliders,
                &mut self.joints,
                &mut self.rigidbody_set,
                gravity,
                timestep_mode,
                events,
                hooks,
                time,
                sim_to_render_time,
                interpolation_query,
            )
        }

        /// Shortcut to [`RapierContextSimulation::move_shape`].
        #[expect(clippy::too_many_arguments)]
        pub fn move_shape(
            &mut self,
            query_pipeline_mut: &mut QueryPipelineMut<'_>,
            movement: Vect,
            shape: &dyn Shape,
            shape_translation: Vect,
            shape_rotation: Rot,
            shape_mass: Real,
            options: &MoveShapeOptions,
            events: impl FnMut(CharacterCollision),
        ) -> MoveShapeOutput {
            self.simulation.move_shape(
                &self.colliders,
                query_pipeline_mut,
                movement,
                shape,
                shape_translation,
                shape_rotation,
                shape_mass,
                options,
                events,
            )
        }

        /// Shortcut to [`RapierContextSimulation::contact_pair`].
        pub fn contact_pair(
            &self,
            collider1: Entity,
            collider2: Entity,
        ) -> Option<ContactPairView<'_>> {
            self.simulation
                .contact_pair(&self.colliders, &self.rigidbody_set, collider1, collider2)
        }

        /// Shortcut to [`RapierContextSimulation::contact_pairs_with`].
        pub fn contact_pairs_with(
            &self,
            collider: Entity,
        ) -> impl Iterator<Item = ContactPairView<'_>> {
            self.simulation
                .contact_pairs_with(&self.colliders, &self.rigidbody_set, collider)
        }

        /// Shortcut to [`RapierContextSimulation::intersection_pair`].
        pub fn intersection_pair(&self, collider1: Entity, collider2: Entity) -> Option<bool> {
            self.simulation
                .intersection_pair(&self.colliders, collider1, collider2)
        }

        /// Shortcut to [`RapierContextSimulation::intersection_pairs_with`].
        pub fn intersection_pairs_with(
            &self,
            collider: Entity,
        ) -> impl Iterator<Item = (Entity, Entity, bool)> + '_ {
            self.simulation
                .intersection_pairs_with(&self.colliders, collider)
        }
    }
}

mod query_pipeline {
    use rapier::parry::query::DefaultQueryDispatcher;

    use crate::prelude::QueryFilter;

    use super::*;

    impl RapierContext<'_> {
        /// Shortcut to [RapierQueryPipeline::new_scoped].
        pub fn with_query_pipeline<'a, T>(
            &'a self,
            filter: QueryFilter<'a>,
            scoped_fn: impl FnOnce(RapierQueryPipeline<'_>) -> T,
        ) -> T {
            crate::prelude::RapierQueryPipeline::new_scoped(
                &self.simulation.broad_phase,
                self.colliders,
                self.rigidbody_set,
                &filter,
                &DefaultQueryDispatcher,
                scoped_fn,
            )
        }
    }

    impl RapierContextMut<'_> {
        /// Shortcut to [RapierQueryPipeline::new_scoped].
        pub fn with_query_pipeline<'a, T>(
            &'a self,
            filter: QueryFilter<'a>,
            scoped_fn: impl FnOnce(RapierQueryPipeline<'_>) -> T,
        ) -> T {
            crate::prelude::RapierQueryPipeline::new_scoped(
                &self.simulation.broad_phase,
                &self.colliders,
                &self.rigidbody_set,
                &filter,
                &DefaultQueryDispatcher,
                scoped_fn,
            )
        }
    }
}

mod rigidbody_set {
    use std::collections::HashMap;

    use super::*;
    pub use rapier::prelude::RigidBodyHandle;

    impl RapierContext<'_> {
        /// Shortcut to [`RapierRigidBodySet::entity2body`].
        pub fn entity2body(&self) -> &HashMap<Entity, RigidBodyHandle> {
            self.rigidbody_set.entity2body()
        }

        /// Shortcut to [`RapierRigidBodySet::rigid_body_entity`].
        pub fn rigid_body_entity(&self, handle: RigidBodyHandle) -> Option<Entity> {
            self.rigidbody_set.rigid_body_entity(handle)
        }

        /// Shortcut to [`RapierRigidBodySet::impulse_revolute_joint_angle`].
        pub fn impulse_revolute_joint_angle(&self, entity: Entity) -> Option<f32> {
            self.rigidbody_set
                .impulse_revolute_joint_angle(self.joints, entity)
        }
    }

    impl RapierContextMut<'_> {
        /// Shortcut to [`RapierRigidBodySet::propagate_modified_body_positions_to_colliders`].
        pub fn propagate_modified_body_positions_to_colliders(&mut self) {
            self.rigidbody_set
                .propagate_modified_body_positions_to_colliders(&mut self.colliders)
        }

        /// Shortcut to [`RapierRigidBodySet::entity2body`].
        pub fn entity2body(&self) -> &HashMap<Entity, RigidBodyHandle> {
            self.rigidbody_set.entity2body()
        }

        /// Shortcut to [`RapierRigidBodySet::rigid_body_entity`].
        pub fn rigid_body_entity(&self, handle: RigidBodyHandle) -> Option<Entity> {
            self.rigidbody_set.rigid_body_entity(handle)
        }

        /// Shortcut to [`RapierRigidBodySet::impulse_revolute_joint_angle`].
        pub fn impulse_revolute_joint_angle(&self, entity: Entity) -> Option<f32> {
            self.rigidbody_set
                .impulse_revolute_joint_angle(&self.joints, entity)
        }
    }
}
