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
    pub fn single(&self) -> Result<RapierContext> {
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
    pub fn single(&self) -> Result<RapierContext> {
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
    pub fn single_mut(&mut self) -> Result<RapierContextMut> {
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
        ) -> Option<ContactPairView> {
            self.simulation
                .contact_pair(self.colliders, self.rigidbody_set, collider1, collider2)
        }

        /// Shortcut to [`RapierContextSimulation::contact_pairs_with`].
        pub fn contact_pairs_with(
            &self,
            collider: Entity,
        ) -> impl Iterator<Item = ContactPairView> {
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
            mut query_pipeline_mut: &mut QueryPipelineMut<'_>,
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
                &mut query_pipeline_mut,
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
        ) -> Option<ContactPairView> {
            self.simulation
                .contact_pair(&self.colliders, &self.rigidbody_set, collider1, collider2)
        }

        /// Shortcut to [`RapierContextSimulation::contact_pairs_with`].
        pub fn contact_pairs_with(
            &self,
            collider: Entity,
        ) -> impl Iterator<Item = ContactPairView> {
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
    use rapier::{
        parry::query::{DefaultQueryDispatcher, ShapeCastOptions},
        prelude::{QueryFilter as RapierQueryFilter, Shape},
    };

    use crate::prelude::{PointProjection, RayIntersection, ShapeCastHit};

    use super::*;

    impl RapierContext<'_> {
        /// Shortcut to [BroadPhaseBvh::as_query_pipeline][rapier::prelude::BroadPhaseBvh::as_query_pipeline].
        pub fn query_pipeline<'a>(
            &'a self,
            filter: RapierQueryFilter<'a>,
        ) -> RapierQueryPipeline<'a> {
            let query_pipeline = RapierQueryPipeline {
                query_pipeline: self.simulation.broad_phase.as_query_pipeline(
                    &DefaultQueryDispatcher,
                    &self.rigidbody_set.bodies,
                    &self.colliders.colliders,
                    filter,
                ),
            };
            query_pipeline
        }

        /// Shortcut to [`RapierQueryPipeline::cast_ray`].
        pub fn cast_ray(
            &self,
            query_pipeline: RapierQueryPipeline<'_>,
            ray_origin: Vect,
            ray_dir: Vect,
            max_toi: Real,
            solid: bool,
        ) -> Option<(Entity, Real)> {
            query_pipeline.cast_ray(self.colliders, ray_origin, ray_dir, max_toi, solid)
        }

        /// Shortcut to [`RapierQueryPipeline::cast_ray_and_get_normal`].
        pub fn cast_ray_and_get_normal(
            &self,
            query_pipeline: RapierQueryPipeline<'_>,
            ray_origin: Vect,
            ray_dir: Vect,
            max_toi: Real,
            solid: bool,
        ) -> Option<(Entity, RayIntersection)> {
            query_pipeline.cast_ray_and_get_normal(
                self.colliders,
                ray_origin,
                ray_dir,
                max_toi,
                solid,
            )
        }

        /// Shortcut to [`RapierQueryPipeline::intersections_with_point`].
        pub fn intersections_with_point(
            &self,
            query_pipeline: RapierQueryPipeline<'_>,
            point: Vect,
            // FIXME: find a way to return an iterator?
            mut callback: impl FnMut(Entity) -> bool,
        ) {
            for e in query_pipeline.intersect_point(self.colliders, point) {
                if !(callback)(e) {
                    break;
                }
            }
        }

        /// Shortcut to [`RapierQueryPipeline::intersections_with_ray`].
        pub fn intersections_with_ray(
            &self,
            query_pipeline: RapierQueryPipeline<'_>,
            ray_origin: Vect,
            ray_dir: Vect,
            max_toi: Real,
            solid: bool,
            // FIXME: find a way to return an iterator?
            mut callback: impl FnMut(Entity, RayIntersection) -> bool,
        ) {
            for (e, intersection) in
                query_pipeline.intersect_ray(self.colliders, ray_origin, ray_dir, max_toi, solid)
            {
                if !(callback)(e, intersection) {
                    break;
                }
            }
        }

        /// Shortcut to [`RapierQueryPipeline::intersections_with_shape`].
        pub fn intersections_with_shape(
            &self,
            query_pipeline: RapierQueryPipeline<'_>,
            shape_pos: Vect,
            shape_rot: Rot,
            shape: &dyn Shape,
            mut callback: impl FnMut(Entity) -> bool,
        ) {
            for e in query_pipeline.intersect_shape(self.colliders, shape_pos, shape_rot, shape) {
                if !(callback)(e) {
                    break;
                }
            }
        }

        /// Shortcut to [`RapierQueryPipeline::colliders_with_aabb_intersecting_aabb`].
        pub fn colliders_with_aabb_intersecting_aabb(
            &self,
            query_pipeline: RapierQueryPipeline<'_>,
            #[cfg(feature = "dim2")] aabb: bevy::math::bounding::Aabb2d,
            #[cfg(feature = "dim3")] aabb: bevy::math::bounding::Aabb3d,
            mut callback: impl FnMut(Entity) -> bool,
        ) {
            for e in query_pipeline.intersect_aabb_conservative(self.colliders, aabb) {
                if !(callback)(e) {
                    break;
                }
            }
        }

        /// Shortcut to [`RapierQueryPipeline::cast_shape`].
        pub fn cast_shape(
            &self,
            query_pipeline: RapierQueryPipeline<'_>,
            shape_pos: Vect,
            shape_rot: Rot,
            shape_vel: Vect,
            shape: &dyn Shape,
            options: ShapeCastOptions,
        ) -> Option<(Entity, ShapeCastHit)> {
            query_pipeline.cast_shape(
                self.colliders,
                shape_pos,
                shape_rot,
                shape_vel,
                shape,
                options,
            )
        }

        /// Shortcut to [`RapierQueryPipeline::project_point`].
        pub fn project_point(
            &self,
            query_pipeline: RapierQueryPipeline<'_>,
            point: Vect,
            max_dist: Real,
            solid: bool,
        ) -> Option<(Entity, PointProjection)> {
            query_pipeline.project_point(self.colliders, point, max_dist, solid)
        }
    }

    impl RapierContextMut<'_> {
        /// Shortcut to [`RapierQueryPipeline::cast_ray`].
        pub fn cast_ray(
            &self,
            query_pipeline: RapierQueryPipeline<'_>,
            ray_origin: Vect,
            ray_dir: Vect,
            max_toi: Real,
            solid: bool,
        ) -> Option<(Entity, Real)> {
            query_pipeline.cast_ray(&self.colliders, ray_origin, ray_dir, max_toi, solid)
        }

        /// Shortcut to [`RapierQueryPipeline::cast_ray_and_get_normal`].
        pub fn cast_ray_and_get_normal(
            &self,
            query_pipeline: RapierQueryPipeline<'_>,
            ray_origin: Vect,
            ray_dir: Vect,
            max_toi: Real,
            solid: bool,
        ) -> Option<(Entity, RayIntersection)> {
            query_pipeline.cast_ray_and_get_normal(
                &self.colliders,
                ray_origin,
                ray_dir,
                max_toi,
                solid,
            )
        }

        /// Shortcut to [`RapierQueryPipeline::intersections_with_point`].
        pub fn intersections_with_point(
            &self,
            query_pipeline: RapierQueryPipeline<'_>,
            point: Vect,
            mut callback: impl FnMut(Entity) -> bool,
        ) {
            for e in query_pipeline.intersect_point(&self.colliders, point) {
                if !(callback)(e) {
                    break;
                }
            }
        }

        /// Shortcut to [`RapierQueryPipeline::intersections_with_ray`].
        pub fn intersections_with_ray(
            &self,
            query_pipeline: RapierQueryPipeline<'_>,
            ray_origin: Vect,
            ray_dir: Vect,
            max_toi: Real,
            solid: bool,
            mut callback: impl FnMut(Entity, RayIntersection) -> bool,
        ) {
            for (e, intersection) in
                query_pipeline.intersect_ray(&self.colliders, ray_origin, ray_dir, max_toi, solid)
            {
                if !(callback)(e, intersection) {
                    break;
                }
            }
        }

        /// Shortcut to [`RapierQueryPipeline::intersections_with_shape`].
        pub fn intersections_with_shape(
            &self,
            query_pipeline: RapierQueryPipeline<'_>,
            shape_pos: Vect,
            shape_rot: Rot,
            shape: &dyn Shape,
            mut callback: impl FnMut(Entity) -> bool,
        ) {
            for e in query_pipeline.intersect_shape(&self.colliders, shape_pos, shape_rot, shape) {
                if !(callback)(e) {
                    break;
                }
            }
        }

        /// Shortcut to [`RapierQueryPipeline::colliders_with_aabb_intersecting_aabb`].
        pub fn colliders_with_aabb_intersecting_aabb(
            &self,
            query_pipeline: RapierQueryPipeline<'_>,
            #[cfg(feature = "dim2")] aabb: bevy::math::bounding::Aabb2d,
            #[cfg(feature = "dim3")] aabb: bevy::math::bounding::Aabb3d,
            mut callback: impl FnMut(Entity) -> bool,
        ) {
            for e in query_pipeline.intersect_aabb_conservative(&self.colliders, aabb) {
                if !(callback)(e) {
                    break;
                }
            }
        }

        /// Shortcut to [`RapierQueryPipeline::cast_shape`].
        pub fn cast_shape(
            &self,
            query_pipeline: RapierQueryPipeline<'_>,
            shape_pos: Vect,
            shape_rot: Rot,
            shape_vel: Vect,
            shape: &dyn Shape,
            options: ShapeCastOptions,
        ) -> Option<(Entity, ShapeCastHit)> {
            query_pipeline.cast_shape(
                &self.colliders,
                shape_pos,
                shape_rot,
                shape_vel,
                shape,
                options,
            )
        }

        /// Shortcut to [`RapierQueryPipeline::project_point`].
        pub fn project_point(
            &self,
            query_pipeline: RapierQueryPipeline<'_>,
            point: Vect,
            max_dist: Real,
            solid: bool,
        ) -> Option<(Entity, PointProjection)> {
            query_pipeline.project_point(&self.colliders, point, max_dist, solid)
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
