use crate::{
    math::{Rot, Vect},
    plugin::context::RapierRigidBodySet,
};
use bevy::ecs::{query, system::SystemParam};
use bevy::prelude::*;
use rapier::prelude::Real;

pub(crate) const RAPIER_CONTEXT_EXPECT_ERROR: &str =
    "RapierContextEntityLink.0 refers to an entity without RapierContext.";

use crate::{
    plugin::context::{RapierContextColliders, RapierContextJoints, RapierQueryPipeline},
    prelude::QueryFilter,
};

use super::super::{DefaultRapierContext, RapierContextSimulation};

macro_rules! define_forwarding_function {
    (
        $fn_name:ident,
        $caller:ident: $caller_type:ty,
        [ $( $self_params:ident ),* ],  // Optional self elements
        [$( $param_name:ident : $param_type:ty ),*$(,)?]
        , $return_value:ty $(,)?
    ) => {
        /// Shortcut to [`$caller_type::$fn_name`]
        pub fn $fn_name(
            &self,
            $( $param_name : $param_type ),*
        ) -> $return_value {
            self.$caller.$fn_name(
                $(
                    &self.$self_params,  // Inject optional self elements (colliders, joints, etc.)
                )*
                $( $param_name ),*  // Forward the other parameters
            )
        }
    };
}

macro_rules! define_forwarding_function_mut {
    (
        $fn_name:ident,
        $caller:ident: $caller_type:ty,
        [ $( $self_params:ident ),* ],  // Optional self elements
        [$( $param_name:ident : $param_type:ty ),*$(,)?]
        , $return_value:ty $(,)?
    ) => {
        /// Shortcut to [`$caller_type::$fn_name`]
        pub fn $fn_name(
            &mut self,
            $( $param_name : $param_type ),*
        ) -> $return_value {
            self.$caller.$fn_name(
                $(
                    &mut self.$self_params,  // Inject optional self elements (colliders, joints, etc.)
                )*
                $( $param_name ),*  // Forward the other parameters
            )
        }
    };
}

/// Utility [`SystemParam`] to easily access every required components of a [`RapierContext`] immutably.
///
/// SAFETY: Dereferencing this struct will panic if its underlying query fails.<br />
/// Use [`ReadRapierContext::rapier_context`] query,
/// or a regular bevy query instead of this [`ReadRapierContext`] for a safer alternative.
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
    /// Use the underlying query [`ReadRapierContext::rapier_context`] for safer alternatives.
    pub fn single(&'_ self) -> RapierContext {
        let (simulation, colliders, joints, query_pipeline, rigidbody_set) =
            self.rapier_context.single();
        RapierContext {
            simulation,
            colliders,
            joints,
            query_pipeline,
            rigidbody_set: rigidbody_set,
        }
    }
}

/// A helper struct to avoid passing too many parameters to most rapier functions.
/// This helps with reducing boilerplate, at the (small) price of maybe getting too much information from the ECS.
///
/// Note: This is not a component, refer to [`ReadRapierContext`] or [`WriteRapierContext`]
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

/// Utility [`SystemParam`] to easily access every required components of a [`RapierContext`] immutably.
///
/// SAFETY: Dereferencing this struct will panic if its underlying query fails.<br />
/// Use [`ReadRapierContext::rapier_context`] query,
/// or a regular bevy [`Query`] instead of this [`ReadRapierContext`] for a safer alternative.
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
            rigidbody_set: rigidbody_set,
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
            rigidbody_set: rigidbody_set,
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
    use crate::plugin::TimestepMode;
    use crate::prelude::Collider;
    use crate::prelude::CollisionEvent;
    use crate::prelude::ContactForceEvent;
    use crate::prelude::RapierRigidBodyHandle;
    use crate::prelude::TransformInterpolation;
    use rapier::prelude::PhysicsHooks;

    use super::*;

    /// [`RapierContextSimulation`] functions for mutable accesses
    impl<'a> RapierContextMut<'a> {
        define_forwarding_function_mut!(step_simulation, simulation: RapierContextSimulation, [colliders, joints, rigidbody_set], [gravity: Vect,
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
            >], ());

        define_forwarding_function_mut!(move_shape, simulation: RapierContextSimulation, [colliders, query_pipeline, rigidbody_set],
            [movement: Vect,
            shape: &Collider,
            shape_translation: Vect,
            shape_rotation: Rot,
            shape_mass: Real,
            options: &MoveShapeOptions,
            filter: QueryFilter,
            events: impl FnMut(CharacterCollision)], MoveShapeOutput);
    }
}

mod query_pipeline {
    use rapier::prelude::QueryFilter as RapierQueryFilter;

    use crate::prelude::{Collider, RayIntersection};

    use super::*;

    impl<'a> RapierContext<'a> {
        define_forwarding_function!(cast_ray, query_pipeline: RapierQueryPipeline, [colliders, rigidbody_set], [ray_origin: Vect,
            ray_dir: Vect,
            max_toi: Real,
            solid: bool,
            filter: QueryFilter], Option<(Entity, Real)>);

        define_forwarding_function!(cast_ray_and_get_normal, query_pipeline: RapierQueryPipeline, [colliders, rigidbody_set], [ray_origin: Vect,
            ray_dir: Vect,
            max_toi: Real,
            solid: bool,
            filter: QueryFilter], Option<(Entity, RayIntersection)>);
    }

    impl<'a> RapierContextMut<'a> {
        define_forwarding_function!(cast_ray, query_pipeline:RapierQueryPipeline, [colliders, rigidbody_set], [ray_origin: Vect,
            ray_dir: Vect,
            max_toi: Real,
            solid: bool,
            filter: QueryFilter,], Option<(Entity, Real)>);

        define_forwarding_function!(cast_ray_and_get_normal, query_pipeline:RapierQueryPipeline, [colliders, rigidbody_set], [ray_origin: Vect,
            ray_dir: Vect,
            max_toi: Real,
            solid: bool,
            filter: QueryFilter], Option<(Entity, RayIntersection)>);

        define_forwarding_function!(intersections_with_ray, query_pipeline:RapierQueryPipeline, [colliders, rigidbody_set], [ray_origin: Vect,
            ray_dir: Vect,
            max_toi: Real,
            solid: bool,
            filter: QueryFilter,
            callback: impl FnMut(Entity, RayIntersection) -> bool,], ());

        define_forwarding_function!(intersections_with_shape, query_pipeline:RapierQueryPipeline, [colliders, rigidbody_set], [
            shape_pos: Vect,
            shape_rot: Rot,
            shape: &Collider,
            filter: QueryFilter,
            callback: impl FnMut(Entity) -> bool,], ());

        // NOTE: `with_query_filter_elts` doesn´t use macro as it's a bit difficult to make it support generics.
        /// Shortcut to [`RapierQueryPipeline::with_query_filter_elts`]
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
    use rapier::prelude::RigidBodyHandle;

    impl<'a> RapierContext<'a> {
        define_forwarding_function!(entity2body, rigidbody_set:RigidBodySet, [], [], &HashMap<Entity, RigidBodyHandle>);

        define_forwarding_function!(rigid_body_entity, rigidbody_set:RigidBodySet, [], [handle: RigidBodyHandle], Option<Entity>);

        define_forwarding_function!(impulse_revolute_joint_angle, rigidbody_set:RigidBodySet, [joints], [entity: Entity], Option<f32>);
    }

    impl<'a> RapierContextMut<'a> {
        define_forwarding_function_mut!(
            propagate_modified_body_positions_to_colliders,
            rigidbody_set:RigidBodySet,
            [colliders],
            [],
            ()
        );

        define_forwarding_function!(entity2body, rigidbody_set:RigidBodySet, [], [], &HashMap<Entity, RigidBodyHandle>);

        define_forwarding_function!(rigid_body_entity, rigidbody_set:RigidBodySet, [], [handle: RigidBodyHandle], Option<Entity>);

        define_forwarding_function!(impulse_revolute_joint_angle, rigidbody_set:RigidBodySet, [joints], [entity: Entity], Option<f32>);
    }
}
