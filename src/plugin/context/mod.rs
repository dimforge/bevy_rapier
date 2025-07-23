//! These are components used and modified during a simulation frame.

pub mod systemparams;

use bevy::prelude::*;
use std::collections::HashMap;
use std::sync::RwLock;

use rapier::prelude::{
    Aabb, CCDSolver, ColliderHandle, ColliderSet, EventHandler, FeatureId, ImpulseJointHandle,
    ImpulseJointSet, IntegrationParameters, IslandManager, MultibodyJointHandle, MultibodyJointSet,
    NarrowPhase, PhysicsHooks, PhysicsPipeline, QueryPipeline, QueryPipelineMut, Ray, Real,
    RigidBodyHandle, RigidBodySet, Shape,
};

use crate::geometry::{PointProjection, RayIntersection, ShapeCastHit};
use crate::math::{Rot, Vect};
use crate::pipeline::{CollisionEvent, ContactForceEvent, EventQueue};
use bevy::prelude::{Entity, EventWriter, GlobalTransform, Query};

use crate::control::{CharacterCollision, MoveShapeOptions, MoveShapeOutput};
use crate::dynamics::TransformInterpolation;
use crate::parry::query::details::ShapeCastOptions;
use crate::plugin::configuration::TimestepMode;
use crate::prelude::RapierRigidBodyHandle;
use rapier::control::CharacterAutostep;
use rapier::geometry::DefaultBroadPhase;

#[cfg(doc)]
use crate::prelude::{
    systemparams::{RapierContext, ReadRapierContext},
    ImpulseJoint, MultibodyJoint, RevoluteJoint, TypedJoint,
};

/// Difference between simulation and rendering time
#[derive(Component, Default, Reflect, Clone)]
pub struct SimulationToRenderTime {
    /// Difference between simulation and rendering time
    pub diff: f32,
}

/// Marker component for to access the default [`ReadRapierContext`].
///
/// This is used as the default marker filter for [`systemparams::ReadRapierContext`] and [`systemparams::WriteRapierContext`]
/// to help with getting a reference to the correct RapierContext.
///
/// If you're making a library, you might be interested in [`RapierContextEntityLink`]
/// and leverage a [`Query`] to have precise access to relevant components (for example [`RapierContextSimulation`]).
///
/// See the list of full components in [`RapierContext`]
#[derive(Component, Reflect, Debug, Clone, Copy)]
pub struct DefaultRapierContext;

/// This is a component applied to any entity containing a rapier handle component.
/// The inner Entity referred to has the component [`RapierContextSimulation`]
/// and others from [`crate::plugin::context`], responsible for handling
/// its rapier data.
#[derive(Component, Reflect, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct RapierContextEntityLink(pub Entity);

/// The set of colliders part of the simulation.
///
/// This should be attached on an entity with a [`RapierContextSimulation`]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Component, Default, Debug, Clone)]
pub struct RapierContextColliders {
    /// The set of colliders part of the simulation.
    pub colliders: ColliderSet,
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub(crate) entity2collider: HashMap<Entity, ColliderHandle>,
}

impl RapierContextColliders {
    /// If the collider attached to `entity` is attached to a rigid-body, this
    /// returns the `Entity` containing that rigid-body.
    pub fn collider_parent(
        &self,
        rigidbody_set: &RapierRigidBodySet,
        entity: Entity,
    ) -> Option<Entity> {
        self.entity2collider
            .get(&entity)
            .and_then(|h| self.colliders.get(*h))
            .and_then(|co| co.parent())
            .and_then(|h| rigidbody_set.rigid_body_entity(h))
    }

    /// If entity is a rigid-body, this returns the collider `Entity`s attached
    /// to that rigid-body.
    pub fn rigid_body_colliders<'a, 'b: 'a>(
        &'a self,
        entity: Entity,
        rigidbody_set: &'b RapierRigidBodySet,
    ) -> impl Iterator<Item = Entity> + 'a {
        rigidbody_set
            .entity2body()
            .get(&entity)
            .and_then(|handle| rigidbody_set.bodies.get(*handle))
            .map(|body| {
                body.colliders()
                    .iter()
                    .filter_map(|handle| self.collider_entity(*handle))
            })
            .into_iter()
            .flatten()
    }

    /// Retrieve the Bevy entity the given Rapier collider (identified by its handle) is attached.
    pub fn collider_entity(&self, handle: ColliderHandle) -> Option<Entity> {
        Self::collider_entity_with_set(&self.colliders, handle)
    }

    // Mostly used to avoid borrowing self completely.
    pub(crate) fn collider_entity_with_set(
        colliders: &ColliderSet,
        handle: ColliderHandle,
    ) -> Option<Entity> {
        colliders
            .get(handle)
            .map(|c| Entity::from_bits(c.user_data as u64))
    }

    /// The map from entities to collider handles.
    pub fn entity2collider(&self) -> &HashMap<Entity, ColliderHandle> {
        &self.entity2collider
    }
}

/// The sets of joints part of the simulation.
///
/// This should be attached on an entity with a [`RapierContextSimulation`]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Component, Default, Debug, Clone)]
pub struct RapierContextJoints {
    /// The set of impulse joints part of the simulation.
    pub impulse_joints: ImpulseJointSet,
    /// The set of multibody joints part of the simulation.
    pub multibody_joints: MultibodyJointSet,

    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub(crate) entity2impulse_joint: HashMap<Entity, ImpulseJointHandle>,
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub(crate) entity2multibody_joint: HashMap<Entity, MultibodyJointHandle>,
}

impl RapierContextJoints {
    /// The map from entities to impulse joint handles.
    pub fn entity2impulse_joint(&self) -> &HashMap<Entity, ImpulseJointHandle> {
        &self.entity2impulse_joint
    }

    /// The map from entities to multibody joint handles.
    pub fn entity2multibody_joint(&self) -> &HashMap<Entity, MultibodyJointHandle> {
        &self.entity2multibody_joint
    }
}

/// Wrapper around [QueryPipeline] to provide bevy friendly methods.
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Copy, Clone)]
pub struct RapierQueryPipeline<'a> {
    /// The query pipeline, which performs scene queries (ray-casting, point projection, etc.)
    pub query_pipeline: QueryPipeline<'a>,
}

/// Wrapper around [QueryPipeline] to provide bevy friendly methods.
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct RapierQueryPipelineMut<'a> {
    /// The query pipeline, which performs scene queries (ray-casting, point projection, etc.)
    pub query_pipeline: QueryPipelineMut<'a>,
}

impl RapierQueryPipelineMut<'_> {
    /// Downgrades the mutable reference to an immutable reference.
    pub fn as_ref(&self) -> RapierQueryPipeline<'_> {
        RapierQueryPipeline {
            query_pipeline: self.query_pipeline.as_ref(),
        }
    }
}

impl<'a> RapierQueryPipeline<'a> {
    /// Find the closest intersection between a ray and a set of collider.
    ///
    /// # Parameters
    /// * `ray_origin`: the starting point of the ray to cast.
    /// * `ray_dir`: the direction of the ray to cast.
    /// * `max_toi`: the maximum time-of-impact that can be reported by this cast. This effectively
    ///   limits the length of the ray to `ray.dir.norm() * max_toi`. Use `Real::MAX` for an unbounded ray.
    /// * `solid`: if this is `true` an impact at time 0.0 (i.e. at the ray origin) is returned if
    ///   it starts inside of a shape. If this `false` then the ray will hit the shape's boundary
    ///   even if its starts inside of it.
    pub fn cast_ray(
        &self,
        rapier_colliders: &RapierContextColliders,
        ray_origin: Vect,
        ray_dir: Vect,
        max_toi: Real,
        solid: bool,
    ) -> Option<(Entity, Real)> {
        let ray = Ray::new(ray_origin.into(), ray_dir.into());

        let (h, toi) = self.query_pipeline.cast_ray(&ray, max_toi, solid)?;

        rapier_colliders.collider_entity(h).map(|e| (e, toi))
    }

    /// Find the closest intersection between a ray and a set of collider.
    ///
    /// # Parameters
    /// * `ray_origin`: the starting point of the ray to cast.
    /// * `ray_dir`: the direction of the ray to cast.
    /// * `max_toi`: the maximum time-of-impact that can be reported by this cast. This effectively
    ///   limits the length of the ray to `ray.dir.norm() * max_toi`. Use `Real::MAX` for an unbounded ray.
    /// * `solid`: if this is `true` an impact at time 0.0 (i.e. at the ray origin) is returned if
    ///   it starts inside of a shape. If this `false` then the ray will hit the shape's boundary
    ///   even if its starts inside of it.
    pub fn cast_ray_and_get_normal(
        &self,
        rapier_colliders: &RapierContextColliders,
        ray_origin: Vect,
        ray_dir: Vect,
        max_toi: Real,
        solid: bool,
    ) -> Option<(Entity, RayIntersection)> {
        let ray = Ray::new(ray_origin.into(), ray_dir.into());

        let (h, result) = self
            .query_pipeline
            .cast_ray_and_get_normal(&ray, max_toi, solid)?;

        rapier_colliders
            .collider_entity(h)
            .map(|e| (e, RayIntersection::from_rapier(result, ray_origin, ray_dir)))
    }

    /// Iterates through all the colliders intersecting a given ray.
    ///
    /// # Parameters
    /// * `ray_origin`: the starting point of the ray to cast.
    /// * `ray_dir`: the direction of the ray to cast.
    /// * `max_toi`: the maximum time-of-impact that can be reported by this cast. This effectively
    ///   limits the length of the ray to `ray.dir.norm() * max_toi`. Use `Real::MAX` for an unbounded ray.
    /// * `solid`: if this is `true` an impact at time 0.0 (i.e. at the ray origin) is returned if
    ///   it starts inside of a shape. If this `false` then the ray will hit the shape's boundary
    ///   even if its starts inside of it.
    #[allow(clippy::too_many_arguments)]
    pub fn intersect_ray(
        &'a self,
        rapier_colliders: &'a RapierContextColliders,
        ray_origin: Vect,
        ray_dir: Vect,
        max_toi: Real,
        solid: bool,
    ) -> impl Iterator<Item = (Entity, RayIntersection)> + 'a {
        let ray = Ray::new(ray_origin.into(), ray_dir.into());

        self.query_pipeline.intersect_ray(ray, max_toi, solid).map(
            move |(collider_handle, _, intersection)| {
                (
                    rapier_colliders.collider_entity(collider_handle).unwrap(),
                    RayIntersection::from_rapier(intersection, ray_origin, ray_dir),
                )
            },
        )
    }

    /// Retrieve all the colliders intersecting the given shape.
    ///
    /// # Parameters
    /// * `shape_pos` - The position of the shape used for the intersection test.
    /// * `shape` - The shape used for the intersection test.
    pub fn intersect_shape(
        &'a self,
        rapier_colliders: &'a RapierContextColliders,
        shape_pos: Vect,
        shape_rot: Rot,
        shape: &'a dyn Shape,
    ) -> impl Iterator<Item = Entity> + 'a {
        let scaled_transform = (shape_pos, shape_rot).into();

        self.query_pipeline
            .intersect_shape(scaled_transform, shape)
            .map(move |(collider_handle, _)| {
                rapier_colliders.collider_entity(collider_handle).unwrap()
            })
    }

    /// Find the projection of a point on the closest collider.
    ///
    /// # Parameters
    /// * `point` - The point to project.
    /// * `solid` - If this is set to `true` then the collider shapes are considered to
    ///   be plain (if the point is located inside of a plain shape, its projection is the point
    ///   itself). If it is set to `false` the collider shapes are considered to be hollow
    ///   (if the point is located inside of an hollow shape, it is projected on the shape's
    ///   boundary).
    pub fn project_point(
        &self,
        rapier_colliders: &RapierContextColliders,
        point: Vect,
        max_dist: Real,
        solid: bool,
    ) -> Option<(Entity, PointProjection)> {
        let (h, result) = self
            .query_pipeline
            .project_point(&point.into(), max_dist, solid)?;

        rapier_colliders
            .collider_entity(h)
            .map(|e| (e, PointProjection::from_rapier(result)))
    }

    /// Find all the colliders containing the given point.
    ///
    /// # Parameters
    /// * `point` - The point used for the containment test.
    pub fn intersect_point(
        &'a self,
        rapier_colliders: &'a RapierContextColliders,
        point: Vect,
    ) -> impl Iterator<Item = Entity> + 'a {
        self.query_pipeline
            .intersect_point(point.into())
            .map(move |(collider_handle, _)| {
                rapier_colliders.collider_entity(collider_handle).unwrap()
            })
    }

    /// Find the projection of a point on the closest collider.
    ///
    /// The results include the ID of the feature hit by the point.
    ///
    /// # Parameters
    /// * `point` - The point to project.
    /// * `solid` - If this is set to `true` then the collider shapes are considered to
    ///   be plain (if the point is located inside of a plain shape, its projection is the point
    ///   itself). If it is set to `false` the collider shapes are considered to be hollow
    ///   (if the point is located inside of an hollow shape, it is projected on the shape's
    ///   boundary).
    pub fn project_point_and_get_feature(
        &self,
        rapier_colliders: &RapierContextColliders,
        point: Vect,
    ) -> Option<(Entity, PointProjection, FeatureId)> {
        let (h, proj, fid) = self
            .query_pipeline
            .project_point_and_get_feature(&point.into())?;

        rapier_colliders
            .collider_entity(h)
            .map(|e| (e, PointProjection::from_rapier(proj), fid))
    }

    /// Finds all handles of all the colliders with an [`Aabb`] intersecting the given [`Aabb`].
    ///
    /// Note that the collider AABB taken into account is the one currently stored in the query
    /// pipeline’s BVH. It doesn’t recompute the latest collider AABB.
    pub fn intersect_aabb_conservative(
        &'a self,
        rapier_colliders: &'a RapierContextColliders,
        #[cfg(feature = "dim2")] aabb: bevy::math::bounding::Aabb2d,
        #[cfg(feature = "dim3")] aabb: bevy::math::bounding::Aabb3d,
    ) -> impl Iterator<Item = Entity> + 'a {
        let scaled_aabb = Aabb {
            mins: aabb.min.into(),
            maxs: aabb.max.into(),
        };
        self.query_pipeline
            .intersect_aabb_conservative(scaled_aabb)
            .map(move |(collider_handle, _)| {
                rapier_colliders.collider_entity(collider_handle).unwrap()
            })
    }

    /// Casts a shape at a constant linear velocity and retrieve the first collider it hits.
    ///
    /// This is similar to ray-casting except that we are casting a whole shape instead of just a
    /// point (the ray origin). In the resulting `ShapeCastHit`, witness and normal 1 refer to the world
    /// collider, and are in world space.
    ///
    /// # Parameters
    /// * `shape_pos` - The initial translation of the shape to cast.
    /// * `shape_rot` - The rotation of the shape to cast.
    /// * `shape_vel` - The constant velocity of the shape to cast (i.e. the cast direction).
    /// * `shape` - The shape to cast.
    /// * `max_toi` - The maximum time-of-impact that can be reported by this cast. This effectively
    ///   limits the distance traveled by the shape to `shape_vel.norm() * maxToi`.
    /// * `stop_at_penetration` - If the casted shape starts in a penetration state with any
    ///   collider, two results are possible. If `stop_at_penetration` is `true` then, the
    ///   result will have a `toi` equal to `start_time`. If `stop_at_penetration` is `false`
    ///   then the nonlinear shape-casting will see if further motion wrt. the penetration normal
    ///   would result in tunnelling. If it does not (i.e. we have a separating velocity along
    ///   that normal) then the nonlinear shape-casting will attempt to find another impact,
    ///   at a time `> start_time` that could result in tunnelling.
    #[allow(clippy::too_many_arguments)]
    pub fn cast_shape(
        &'a self,
        rapier_colliders: &'a RapierContextColliders,
        shape_pos: Vect,
        shape_rot: Rot,
        shape_vel: Vect,
        shape: &dyn Shape,
        options: ShapeCastOptions,
    ) -> Option<(Entity, ShapeCastHit)> {
        let scaled_transform = (shape_pos, shape_rot).into();

        let (h, result) =
            self.query_pipeline
                .cast_shape(&scaled_transform, &shape_vel.into(), shape, options)?;

        rapier_colliders.collider_entity(h).map(|e| {
            (
                e,
                ShapeCastHit::from_rapier(result, options.compute_impact_geometry_on_penetration),
            )
        })
    }

    /* TODO: we need to wrap the NonlinearRigidMotion somehow.
     *
    /// Casts a shape with an arbitrary continuous motion and retrieve the first collider it hits.
    ///
    /// In the resulting `ShapeCastHit`, witness and normal 1 refer to the world collider, and are
    /// in world space.
    ///
    /// # Parameters
    /// * `shape_motion` - The motion of the shape.
    /// * `shape` - The shape to cast.
    /// * `start_time` - The starting time of the interval where the motion takes place.
    /// * `end_time` - The end time of the interval where the motion takes place.
    /// * `stop_at_penetration` - If the casted shape starts in a penetration state with any
    ///    collider, two results are possible. If `stop_at_penetration` is `true` then, the
    ///    result will have a `toi` equal to `start_time`. If `stop_at_penetration` is `false`
    ///    then the nonlinear shape-casting will see if further motion wrt. the penetration normal
    ///    would result in tunnelling. If it does not (i.e. we have a separating velocity along
    ///    that normal) then the nonlinear shape-casting will attempt to find another impact,
    ///    at a time `> start_time` that could result in tunnelling.
    /// * `filter`: set of rules used to determine which collider is taken into account by this scene query.
    pub fn nonlinear_cast_shape(
        &self,
        shape_motion: &NonlinearRigidMotion,
        shape: &Collider,
        start_time: Real,
        end_time: Real,
        stop_at_penetration: bool,
        filter: QueryFilter,
    ) -> Option<(Entity, Toi)> {
        let scaled_transform = (shape_pos, shape_rot).into();
        let mut scaled_shape = shape.clone();
        // TODO: how to set a good number of subdivisions, we don’t have access to the
        //       RapierConfiguration::scaled_shape_subdivision here.
        scaled_shape.set_scale(shape.scale, 20);

        let (h, result) = rigidbody_set.with_query_filter(filter, move |filter| {
            self.query_pipeline.nonlinear_cast_shape(
                &rigidbody_set.bodies,
                &self.colliders,
                shape_motion,
                &*scaled_shape.raw,
                start_time,
                end_time,
                stop_at_penetration,
                filter,
            )
        })?;

        self.collider_entity(h).map(|e| (e, result))
    }
     */
}

/// The set of rigid-bodies part of the simulation.
///
/// This should be attached on an entity with a [`RapierContextSimulation`]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Component, Default, Clone)]
pub struct RapierRigidBodySet {
    /// The set of rigid-bodies part of the simulation.
    pub bodies: RigidBodySet,
    /// NOTE: this map is needed to handle despawning.
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub(crate) entity2body: HashMap<Entity, RigidBodyHandle>,

    /// For transform change detection.
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub(crate) last_body_transform_set: HashMap<RigidBodyHandle, GlobalTransform>,
}

impl RapierRigidBodySet {
    /// The map from entities to rigid-body handles.
    pub fn entity2body(&self) -> &HashMap<Entity, RigidBodyHandle> {
        &self.entity2body
    }

    /// Retrieve the Bevy entity the given Rapier rigid-body (identified by its handle) is attached.
    pub fn rigid_body_entity(&self, handle: RigidBodyHandle) -> Option<Entity> {
        self.bodies
            .get(handle)
            .map(|c| Entity::from_bits(c.user_data as u64))
    }

    /// This method makes sure that the rigid-body positions have been propagated to
    /// their attached colliders, without having to perform a simulation step.
    pub fn propagate_modified_body_positions_to_colliders(
        &self,
        colliders: &mut RapierContextColliders,
    ) {
        self.bodies
            .propagate_modified_body_positions_to_colliders(&mut colliders.colliders);
    }

    /// Computes the angle between the two bodies attached by the [`RevoluteJoint`] component (if any) referenced by the given `entity`.
    ///
    /// The angle is computed along the revolute joint’s principal axis.
    ///
    /// Parameter `entity` should have a [`ImpulseJoint`] component with a [`TypedJoint::RevoluteJoint`] variant as `data`.
    pub fn impulse_revolute_joint_angle(
        &self,
        joints: &RapierContextJoints,
        entity: Entity,
    ) -> Option<f32> {
        let joint_handle = joints.entity2impulse_joint().get(&entity)?;
        let impulse_joint = joints.impulse_joints.get(*joint_handle)?;
        let revolute_joint = impulse_joint.data.as_revolute()?;

        let rb1 = &self.bodies[impulse_joint.body1];
        let rb2 = &self.bodies[impulse_joint.body2];
        Some(revolute_joint.angle(rb1.rotation(), rb2.rotation()))
    }
}

/// The Rapier context, containing parts of the state of the physics engine, specific to the simulation step.
///
/// This is the main driver for a rapier context, which will create other required components if needed.
///
/// Additionally to its required components, this component is also always paired with a [`RapierConfiguration`][crate::prelude::RapierConfiguration] component.
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Component)]
#[require(
    RapierContextColliders,
    RapierRigidBodySet,
    RapierContextJoints,
    SimulationToRenderTime
)]
pub struct RapierContextSimulation {
    /// The island manager, which detects what object is sleeping
    /// (not moving much) to reduce computations.
    pub islands: IslandManager,
    /// The broad-phase, which detects potential contact pairs.
    pub broad_phase: DefaultBroadPhase,
    /// The narrow-phase, which computes contact points, tests intersections,
    /// and maintain the contact and intersection graphs.
    pub narrow_phase: NarrowPhase,
    /// The solver, which handles Continuous Collision Detection (CCD).
    pub ccd_solver: CCDSolver,
    /// The physics pipeline, which advance the simulation step by step.
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub pipeline: PhysicsPipeline,
    /// The integration parameters, controlling various low-level coefficient of the simulation.
    pub integration_parameters: IntegrationParameters,
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub(crate) event_handler: Option<Box<dyn EventHandler>>,
    // This maps the handles of colliders that have been deleted since the last
    // physics update, to the entity they was attached to.
    /// NOTE: this map is needed to handle despawning.
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub(crate) deleted_colliders: HashMap<ColliderHandle, Entity>,

    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub(crate) collision_events_to_send: Vec<CollisionEvent>,
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub(crate) contact_force_events_to_send: Vec<ContactForceEvent>,
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub(crate) character_collisions_collector: Vec<rapier::control::CharacterCollision>,
}

impl Default for RapierContextSimulation {
    fn default() -> Self {
        Self {
            islands: IslandManager::new(),
            broad_phase: DefaultBroadPhase::new(),
            narrow_phase: NarrowPhase::new(),
            ccd_solver: CCDSolver::new(),
            pipeline: PhysicsPipeline::new(),
            integration_parameters: IntegrationParameters::default(),
            event_handler: None,
            deleted_colliders: HashMap::default(),
            collision_events_to_send: Vec::new(),
            contact_force_events_to_send: Vec::new(),
            character_collisions_collector: Vec::new(),
        }
    }
}

impl RapierContextSimulation {
    /// Advance the simulation, based on the given timestep mode.
    #[allow(clippy::too_many_arguments)]
    pub fn step_simulation(
        &mut self,
        colliders: &mut RapierContextColliders,
        joints: &mut RapierContextJoints,
        rigidbody_set: &mut RapierRigidBodySet,
        gravity: Vect,
        timestep_mode: TimestepMode,
        events: Option<(
            &EventWriter<CollisionEvent>,
            &EventWriter<ContactForceEvent>,
        )>,
        hooks: &dyn PhysicsHooks,
        time: &Time,
        sim_to_render_time: &mut SimulationToRenderTime,
        mut interpolation_query: Option<
            &mut Query<(&RapierRigidBodyHandle, &mut TransformInterpolation)>,
        >,
    ) {
        let event_queue = if events.is_some() {
            Some(EventQueue {
                deleted_colliders: &self.deleted_colliders,
                collision_events: RwLock::new(Vec::new()),
                contact_force_events: RwLock::new(Vec::new()),
            })
        } else {
            None
        };

        let event_handler = self
            .event_handler
            .as_deref()
            .or_else(|| event_queue.as_ref().map(|q| q as &dyn EventHandler))
            .unwrap_or(&() as &dyn EventHandler);

        let mut executed_steps = 0;
        match timestep_mode {
            TimestepMode::Interpolated {
                dt,
                time_scale,
                substeps,
            } => {
                self.integration_parameters.dt = dt;

                sim_to_render_time.diff += time.delta_secs();

                while sim_to_render_time.diff > 0.0 {
                    // NOTE: in this comparison we do the same computations we
                    // will do for the next `while` iteration test, to make sure we
                    // don't get bit by potential float inaccuracy.
                    if sim_to_render_time.diff - dt <= 0.0 {
                        if let Some(interpolation_query) = interpolation_query.as_mut() {
                            // This is the last simulation step to be executed in the loop
                            // Update the previous state transforms
                            for (handle, mut interpolation) in interpolation_query.iter_mut() {
                                if let Some(body) = rigidbody_set.bodies.get(handle.0) {
                                    interpolation.start = Some(*body.position());
                                    interpolation.end = None;
                                }
                            }
                        }
                    }

                    let mut substep_integration_parameters = self.integration_parameters;
                    substep_integration_parameters.dt = dt / (substeps as Real) * time_scale;

                    for _ in 0..substeps {
                        self.pipeline.step(
                            &gravity.into(),
                            &substep_integration_parameters,
                            &mut self.islands,
                            &mut self.broad_phase,
                            &mut self.narrow_phase,
                            &mut rigidbody_set.bodies,
                            &mut colliders.colliders,
                            &mut joints.impulse_joints,
                            &mut joints.multibody_joints,
                            &mut self.ccd_solver,
                            hooks,
                            event_handler,
                        );
                        executed_steps += 1;
                    }

                    sim_to_render_time.diff -= dt;
                }
            }
            TimestepMode::Variable {
                max_dt,
                time_scale,
                substeps,
            } => {
                self.integration_parameters.dt = (time.delta_secs() * time_scale).min(max_dt);

                let mut substep_integration_parameters = self.integration_parameters;
                substep_integration_parameters.dt /= substeps as Real;

                for _ in 0..substeps {
                    self.pipeline.step(
                        &gravity.into(),
                        &substep_integration_parameters,
                        &mut self.islands,
                        &mut self.broad_phase,
                        &mut self.narrow_phase,
                        &mut rigidbody_set.bodies,
                        &mut colliders.colliders,
                        &mut joints.impulse_joints,
                        &mut joints.multibody_joints,
                        &mut self.ccd_solver,
                        hooks,
                        event_handler,
                    );
                    executed_steps += 1;
                }
            }
            TimestepMode::Fixed { dt, substeps } => {
                self.integration_parameters.dt = dt;

                let mut substep_integration_parameters = self.integration_parameters;
                substep_integration_parameters.dt = dt / (substeps as Real);

                for _ in 0..substeps {
                    self.pipeline.step(
                        &gravity.into(),
                        &substep_integration_parameters,
                        &mut self.islands,
                        &mut self.broad_phase,
                        &mut self.narrow_phase,
                        &mut rigidbody_set.bodies,
                        &mut colliders.colliders,
                        &mut joints.impulse_joints,
                        &mut joints.multibody_joints,
                        &mut self.ccd_solver,
                        hooks,
                        event_handler,
                    );
                    executed_steps += 1;
                }
            }
        }
        if let Some(mut event_queue) = event_queue {
            // NOTE: event_queue and its inner locks are only accessed from
            // within `self.pipeline.step` called above, so we can unwrap here safely.
            self.collision_events_to_send =
                std::mem::take(event_queue.collision_events.get_mut().unwrap());
            self.contact_force_events_to_send =
                std::mem::take(event_queue.contact_force_events.get_mut().unwrap());
        }

        if executed_steps > 0 {
            self.deleted_colliders.clear();
        }
    }
    /// Generates bevy events for any physics interactions that have happened
    /// that are stored in the events list
    pub fn send_bevy_events(
        &mut self,
        collision_event_writer: &mut EventWriter<CollisionEvent>,
        contact_force_event_writer: &mut EventWriter<ContactForceEvent>,
    ) {
        for collision_event in self.collision_events_to_send.drain(..) {
            collision_event_writer.write(collision_event);
        }
        for contact_force_event in self.contact_force_events_to_send.drain(..) {
            contact_force_event_writer.write(contact_force_event);
        }
    }

    /// Attempts to move shape, optionally sliding or climbing obstacles.
    ///
    /// # Parameters
    /// * `movement`: the translational movement to apply.
    /// * `shape`: the shape to move.
    /// * `shape_translation`: the initial position of the shape.
    /// * `shape_rotation`: the rotation of the shape.
    /// * `shape_mass`: the mass of the shape to be considered by the impulse calculation if
    ///   `MoveShapeOptions::apply_impulse_to_dynamic_bodies` is set to true.
    /// * `options`: configures the behavior of the automatic sliding and climbing.
    /// * `events`: callback run on each obstacle hit by the shape on its path.
    #[allow(clippy::too_many_arguments)]
    pub fn move_shape(
        &mut self,
        rapier_colliders: &RapierContextColliders,
        rapier_query_pipeline: &mut QueryPipelineMut,
        movement: Vect,
        shape: &dyn Shape,
        shape_translation: Vect,
        shape_rotation: Rot,
        shape_mass: Real,
        options: &MoveShapeOptions,
        mut events: impl FnMut(CharacterCollision),
    ) -> MoveShapeOutput {
        let up = options
            .up
            .try_into()
            .expect("The up vector must be non-zero.");
        let autostep = options.autostep.map(|autostep| CharacterAutostep {
            max_height: autostep.max_height,
            min_width: autostep.min_width,
            include_dynamic_bodies: autostep.include_dynamic_bodies,
        });
        let controller = rapier::control::KinematicCharacterController {
            up,
            offset: options.offset,
            slide: options.slide,
            autostep,
            max_slope_climb_angle: options.max_slope_climb_angle,
            min_slope_slide_angle: options.min_slope_slide_angle,
            snap_to_ground: options.snap_to_ground,
            normal_nudge_factor: options.normal_nudge_factor,
        };

        self.character_collisions_collector.clear();

        // TODO: having to grab all the references to avoid having self in
        //       the closure is ugly.
        let dt = self.integration_parameters.dt;
        let colliders = &rapier_colliders.colliders;
        let collisions = &mut self.character_collisions_collector;
        collisions.clear();

        let result = controller.move_shape(
            dt,
            &rapier_query_pipeline.as_ref(),
            shape,
            &(shape_translation, shape_rotation).into(),
            movement.into(),
            |c| {
                if let Some(collision) = CharacterCollision::from_raw_with_set(colliders, &c, true)
                {
                    events(collision);
                }
                collisions.push(c);
            },
        );

        if options.apply_impulse_to_dynamic_bodies {
            controller.solve_character_collision_impulses(
                dt,
                rapier_query_pipeline,
                shape,
                shape_mass,
                collisions.iter(),
            )
        };

        MoveShapeOutput {
            effective_translation: result.translation.into(),
            grounded: result.grounded,
            is_sliding_down_slope: result.is_sliding_down_slope,
        }
    }
}
