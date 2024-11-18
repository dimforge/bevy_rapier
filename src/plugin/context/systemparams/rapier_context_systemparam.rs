use crate::math::{Rot, Vect};
use bevy::ecs::{query, system::SystemParam};
use bevy::prelude::*;
use rapier::prelude::Real;

pub(crate) const RAPIER_CONTEXT_EXPECT_ERROR: &str =
    "RapierContextEntityLink.0 refers to an entity missing components from RapierContextBundle.";

use crate::{
    plugin::context::{
        DefaultRapierContext, RapierContextColliders, RapierContextJoints, RapierContextSimulation,
        RapierQueryPipeline, RapierRigidBodySet,
    },
    prelude::QueryFilter,
};

#[cfg(doc)]
use crate::prelude::RapierContextBundle;

/// Utility [`SystemParam`] to easily access every required components of a [`RapierContext`] immutably.
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
            &'static RapierQueryPipeline,
            &'static RapierRigidBodySet,
        ),
        T,
    >,
}

impl<'w, 's, T: query::QueryFilter + 'static> ReadRapierContext<'w, 's, T> {
    /// Use this method if you only have one [`RapierContext`].
    ///
    /// SAFETY: This method will panic if its underlying query fails.
    ///
    /// Use the underlying query [`ReadRapierContext::rapier_context`] for safer alternatives.
    pub fn single(&self) -> RapierContext {
        let (simulation, colliders, joints, query_pipeline, rigidbody_set) =
            self.rapier_context.single();
        RapierContext {
            simulation,
            colliders,
            joints,
            query_pipeline,
            rigidbody_set,
        }
    }
}

/// A helper struct to avoid passing too many parameters to most rapier functions.
/// This helps with reducing boilerplate, at the (small) price of maybe getting too much information from the ECS.
///
/// Note: This is not a component, refer to [`ReadRapierContext`], [`WriteRapierContext`], or [`RapierContextBundle`]
#[derive(query::QueryData)]
pub struct RapierContext<'a> {
    /// The Rapier context, containing all the state of the physics engine.
    pub simulation: &'a RapierContextSimulation,
    /// The set of colliders part of the simulation.
    pub colliders: &'a RapierContextColliders,
    /// The sets of joints part of the simulation.
    pub joints: &'a RapierContextJoints,
    /// The query pipeline, which performs scene queries (ray-casting, point projection, etc.)
    pub query_pipeline: &'a RapierQueryPipeline,
    /// The set of rigid-bodies part of the simulation.
    pub rigidbody_set: &'a RapierRigidBodySet,
}

/// Utility [`SystemParam`] to easily access every required components of a [`RapierContext`] mutably.
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
            &'static mut RapierQueryPipeline,
            &'static mut RapierRigidBodySet,
        ),
        T,
    >,
}

impl<'w, 's, T: query::QueryFilter + 'static> WriteRapierContext<'w, 's, T> {
    /// Use this method if you only have one [`RapierContext`] corresponding to the filter (T) of [`WriteRapierContext`].
    ///
    /// SAFETY: This method will panic if its underlying query fails.
    /// Use the underlying query [`WriteRapierContext::rapier_context`] for safer alternatives.
    pub fn single(&self) -> RapierContext {
        let (simulation, colliders, joints, query_pipeline, rigidbody_set) =
            self.rapier_context.single();
        RapierContext {
            simulation,
            colliders,
            joints,
            query_pipeline,
            rigidbody_set,
        }
    }
    /// Use this method if you only have one [`RapierContext`].
    ///
    /// SAFETY: This method will panic if its underlying query fails.
    /// Use the underlying query [`WriteRapierContext::rapier_context`] for safer alternatives.
    pub fn single_mut(&mut self) -> RapierContextMut {
        let (simulation, colliders, joints, query_pipeline, rigidbody_set) =
            self.rapier_context.single_mut();
        RapierContextMut {
            simulation,
            colliders,
            joints,
            query_pipeline,
            rigidbody_set,
        }
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
    /// The query pipeline, which performs scene queries (ray-casting, point projection, etc.)
    pub query_pipeline: Mut<'a, RapierQueryPipeline>,
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
    use crate::prelude::Collider;
    use crate::prelude::CollisionEvent;
    use crate::prelude::ContactForceEvent;
    use crate::prelude::RapierRigidBodyHandle;
    use crate::prelude::TransformInterpolation;
    use rapier::prelude::PhysicsHooks;

    use super::*;

    /// [`RapierContextSimulation`] functions for immutable accesses
    impl<'a> RapierContext<'a> {
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
    impl<'a> RapierContextMut<'a> {
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
            movement: Vect,
            shape: &Collider,
            shape_translation: Vect,
            shape_rotation: Rot,
            shape_mass: Real,
            options: &MoveShapeOptions,
            filter: QueryFilter,
            events: impl FnMut(CharacterCollision),
        ) -> MoveShapeOutput {
            self.simulation.move_shape(
                &self.colliders,
                &self.query_pipeline,
                &mut self.rigidbody_set,
                movement,
                shape,
                shape_translation,
                shape_rotation,
                shape_mass,
                options,
                filter,
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
    use rapier::{parry::query::ShapeCastOptions, prelude::QueryFilter as RapierQueryFilter};

    use crate::prelude::{Collider, PointProjection, RayIntersection, ShapeCastHit};

    use super::*;

    impl<'a> RapierContext<'a> {
        /// Shortcut to [`RapierQueryPipeline::cast_ray`].
        pub fn cast_ray(
            &self,
            ray_origin: Vect,
            ray_dir: Vect,
            max_toi: Real,
            solid: bool,
            filter: QueryFilter,
        ) -> Option<(Entity, Real)> {
            self.query_pipeline.cast_ray(
                self.colliders,
                self.rigidbody_set,
                ray_origin,
                ray_dir,
                max_toi,
                solid,
                filter,
            )
        }

        /// Shortcut to [`RapierQueryPipeline::cast_ray_and_get_normal`].
        pub fn cast_ray_and_get_normal(
            &self,
            ray_origin: Vect,
            ray_dir: Vect,
            max_toi: Real,
            solid: bool,
            filter: QueryFilter,
        ) -> Option<(Entity, RayIntersection)> {
            self.query_pipeline.cast_ray_and_get_normal(
                self.colliders,
                self.rigidbody_set,
                ray_origin,
                ray_dir,
                max_toi,
                solid,
                filter,
            )
        }

        /// Shortcut to [`RapierQueryPipeline::intersections_with_point`].
        pub fn intersections_with_point(
            &self,
            point: Vect,
            filter: QueryFilter,
            callback: impl FnMut(Entity) -> bool,
        ) {
            self.query_pipeline.intersections_with_point(
                self.colliders,
                self.rigidbody_set,
                point,
                filter,
                callback,
            );
        }

        /// Shortcut to [`RapierQueryPipeline::intersections_with_ray`].
        pub fn intersections_with_ray(
            &self,
            ray_origin: Vect,
            ray_dir: Vect,
            max_toi: Real,
            solid: bool,
            filter: QueryFilter,
            callback: impl FnMut(Entity, RayIntersection) -> bool,
        ) {
            self.query_pipeline.intersections_with_ray(
                self.colliders,
                self.rigidbody_set,
                ray_origin,
                ray_dir,
                max_toi,
                solid,
                filter,
                callback,
            )
        }

        /// Shortcut to [`RapierQueryPipeline::intersections_with_shape`].
        pub fn intersections_with_shape(
            &self,
            shape_pos: Vect,
            shape_rot: Rot,
            shape: &Collider,
            filter: QueryFilter,
            callback: impl FnMut(Entity) -> bool,
        ) {
            self.query_pipeline.intersections_with_shape(
                self.colliders,
                self.rigidbody_set,
                shape_pos,
                shape_rot,
                shape,
                filter,
                callback,
            )
        }

        /// Shortcut to [`RapierQueryPipeline::colliders_with_aabb_intersecting_aabb`].
        pub fn colliders_with_aabb_intersecting_aabb(
            &self,
            #[cfg(feature = "dim2")] aabb: bevy::math::bounding::Aabb2d,
            #[cfg(feature = "dim3")] aabb: bevy::math::bounding::Aabb3d,
            callback: impl FnMut(Entity) -> bool,
        ) {
            self.query_pipeline.colliders_with_aabb_intersecting_aabb(
                &self.colliders,
                aabb,
                callback,
            )
        }

        /// Shortcut to [`RapierQueryPipeline::cast_shape`].
        pub fn cast_shape(
            &self,
            shape_pos: Vect,
            shape_rot: Rot,
            shape_vel: Vect,
            shape: &Collider,
            options: ShapeCastOptions,
            filter: QueryFilter,
        ) -> Option<(Entity, ShapeCastHit)> {
            self.query_pipeline.cast_shape(
                self.colliders,
                self.rigidbody_set,
                shape_pos,
                shape_rot,
                shape_vel,
                shape,
                options,
                filter,
            )
        }

        /// Shortcut to [`RapierQueryPipeline::project_point`].
        pub fn project_point(
            &self,
            point: Vect,
            solid: bool,
            filter: QueryFilter,
        ) -> Option<(Entity, PointProjection)> {
            self.query_pipeline.project_point(
                self.colliders,
                self.rigidbody_set,
                point,
                solid,
                filter,
            )
        }

        /// Shortcut to [`RapierQueryPipeline::with_query_filter_elts`].
        pub fn with_query_filter_elts<T>(
            &self,
            filter: crate::prelude::QueryFilter,
            f: impl FnOnce(RapierQueryFilter) -> T,
        ) -> T {
            RapierQueryPipeline::with_query_filter_elts(
                &self.colliders.entity2collider,
                &self.rigidbody_set.entity2body,
                &self.colliders.colliders,
                filter,
                f,
            )
        }
    }

    impl<'a> RapierContextMut<'a> {
        /// Shortcut to [`RapierQueryPipeline::cast_ray`].
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

        /// Shortcut to [`RapierQueryPipeline::cast_ray_and_get_normal`].
        pub fn cast_ray_and_get_normal(
            &self,
            ray_origin: Vect,
            ray_dir: Vect,
            max_toi: Real,
            solid: bool,
            filter: QueryFilter,
        ) -> Option<(Entity, RayIntersection)> {
            self.query_pipeline.cast_ray_and_get_normal(
                &self.colliders,
                &self.rigidbody_set,
                ray_origin,
                ray_dir,
                max_toi,
                solid,
                filter,
            )
        }

        /// Shortcut to [`RapierQueryPipeline::intersections_with_point`].
        pub fn intersections_with_point(
            &self,
            point: Vect,
            filter: QueryFilter,
            callback: impl FnMut(Entity) -> bool,
        ) {
            self.query_pipeline.intersections_with_point(
                &self.colliders,
                &self.rigidbody_set,
                point,
                filter,
                callback,
            );
        }

        /// Shortcut to [`RapierQueryPipeline::intersections_with_ray`].
        pub fn intersections_with_ray(
            &self,
            ray_origin: Vect,
            ray_dir: Vect,
            max_toi: Real,
            solid: bool,
            filter: QueryFilter,
            callback: impl FnMut(Entity, RayIntersection) -> bool,
        ) {
            self.query_pipeline.intersections_with_ray(
                &self.colliders,
                &self.rigidbody_set,
                ray_origin,
                ray_dir,
                max_toi,
                solid,
                filter,
                callback,
            )
        }

        /// Shortcut to [`RapierQueryPipeline::intersections_with_shape`].
        pub fn intersections_with_shape(
            &self,
            shape_pos: Vect,
            shape_rot: Rot,
            shape: &Collider,
            filter: QueryFilter,
            callback: impl FnMut(Entity) -> bool,
        ) {
            self.query_pipeline.intersections_with_shape(
                &self.colliders,
                &self.rigidbody_set,
                shape_pos,
                shape_rot,
                shape,
                filter,
                callback,
            )
        }

        /// Shortcut to [`RapierQueryPipeline::colliders_with_aabb_intersecting_aabb`].
        pub fn colliders_with_aabb_intersecting_aabb(
            &self,
            #[cfg(feature = "dim2")] aabb: bevy::math::bounding::Aabb2d,
            #[cfg(feature = "dim3")] aabb: bevy::math::bounding::Aabb3d,
            callback: impl FnMut(Entity) -> bool,
        ) {
            self.query_pipeline.colliders_with_aabb_intersecting_aabb(
                &self.colliders,
                aabb,
                callback,
            )
        }

        /// Shortcut to [`RapierQueryPipeline::cast_shape`].
        pub fn cast_shape(
            &self,
            shape_pos: Vect,
            shape_rot: Rot,
            shape_vel: Vect,
            shape: &Collider,
            options: ShapeCastOptions,
            filter: QueryFilter,
        ) -> Option<(Entity, ShapeCastHit)> {
            self.query_pipeline.cast_shape(
                &self.colliders,
                &self.rigidbody_set,
                shape_pos,
                shape_rot,
                shape_vel,
                shape,
                options,
                filter,
            )
        }

        /// Shortcut to [`RapierQueryPipeline::project_point`].
        pub fn project_point(
            &self,
            point: Vect,
            solid: bool,
            filter: QueryFilter,
        ) -> Option<(Entity, PointProjection)> {
            self.query_pipeline.project_point(
                &self.colliders,
                &self.rigidbody_set,
                point,
                solid,
                filter,
            )
        }

        /// Shortcut to [`RapierQueryPipeline::with_query_filter_elts`].
        pub fn with_query_filter_elts<T>(
            &self,
            filter: crate::prelude::QueryFilter,
            f: impl FnOnce(RapierQueryFilter) -> T,
        ) -> T {
            RapierQueryPipeline::with_query_filter_elts(
                &self.colliders.entity2collider,
                &self.rigidbody_set.entity2body,
                &self.colliders.colliders,
                filter,
                f,
            )
        }
    }
}

mod rigidbody_set {
    use std::collections::HashMap;

    use super::*;
    pub use rapier::prelude::RigidBodyHandle;

    impl<'a> RapierContext<'a> {
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

    impl<'a> RapierContextMut<'a> {
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
