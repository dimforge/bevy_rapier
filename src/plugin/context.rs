use bevy::core::Time;
use std::collections::HashMap;
use std::sync::RwLock;

use rapier::prelude::*;

use crate::geometry::{Collider, InteractionGroups, PointProjection, RayIntersection, Toi};
use crate::math::{Rot, Vect};
use crate::pipeline::{CollisionEvent, EventQueue};
use bevy::prelude::{Entity, EventWriter, GlobalTransform, Query};
use bevy::render::primitives::Aabb;

use crate::dynamics::TransformInterpolation;
use crate::plugin::configuration::{SimulationToRenderTime, TimestepMode};
use crate::prelude::RapierRigidBodyHandle;
#[cfg(feature = "dim2")]
use bevy::math::Vec3Swizzles;

/// The Rapier context, containing all the state of the physics engine.
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct RapierContext {
    /// The island manager, which detects what object is sleeping
    /// (not moving much) to reduce computations.
    pub islands: IslandManager,
    /// The broad-phase, which detects potential contact pairs.
    pub broad_phase: BroadPhase,
    /// The narrow-phase, which computes contact points, tests intersections,
    /// and maintain the contact and intersection graphs.
    pub narrow_phase: NarrowPhase,
    /// The set of rigid-bodies part of the simulation.
    pub bodies: RigidBodySet,
    /// The set of colliders part of the simulation.
    pub colliders: ColliderSet,
    /// The set of impulse joints part of the simulation.
    pub impulse_joints: ImpulseJointSet,
    /// The set of multibody joints part of the simulation.
    pub multibody_joints: MultibodyJointSet,
    /// The solver, which handles Continuous Collision Detection (CCD).
    pub ccd_solver: CCDSolver,
    /// The physics pipeline, which advance the simulation step by step.
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub pipeline: PhysicsPipeline,
    /// The query pipeline, which performs scene queries (ray-casting, point projection, etc.)
    pub query_pipeline: QueryPipeline,
    /// The integration parameters, controlling various low-level coefficient of the simulation.
    pub integration_parameters: IntegrationParameters,
    pub(crate) physics_scale: Real,
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub(crate) event_handler: Option<Box<dyn EventHandler>>,
    // For transform change detection.
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub(crate) last_body_transform_set: HashMap<RigidBodyHandle, GlobalTransform>,
    // NOTE: these maps are needed to handle despawning.
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub(crate) entity2body: HashMap<Entity, RigidBodyHandle>,
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub(crate) entity2collider: HashMap<Entity, ColliderHandle>,
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub(crate) entity2impulse_joint: HashMap<Entity, ImpulseJointHandle>,
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub(crate) entity2multibody_joint: HashMap<Entity, MultibodyJointHandle>,
    // This maps the handles of colliders that have been deleted since the last
    // physics update, to the entity they was attached to.
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub(crate) deleted_colliders: HashMap<ColliderHandle, Entity>,
}

impl Default for RapierContext {
    fn default() -> Self {
        Self {
            islands: IslandManager::new(),
            broad_phase: BroadPhase::new(),
            narrow_phase: NarrowPhase::new(),
            bodies: RigidBodySet::new(),
            colliders: ColliderSet::new(),
            impulse_joints: ImpulseJointSet::new(),
            multibody_joints: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            pipeline: PhysicsPipeline::new(),
            query_pipeline: QueryPipeline::new(),
            integration_parameters: IntegrationParameters::default(),
            physics_scale: 1.0,
            event_handler: None,
            last_body_transform_set: HashMap::new(),
            entity2body: HashMap::new(),
            entity2collider: HashMap::new(),
            entity2impulse_joint: HashMap::new(),
            entity2multibody_joint: HashMap::new(),
            deleted_colliders: HashMap::new(),
        }
    }
}

impl RapierContext {
    /// If the collider attached to `entity` is attached to a rigid-body, this
    /// returns the `Entity` containing that rigid-body.
    pub fn collider_parent(&self, entity: Entity) -> Option<Entity> {
        self.entity2collider
            .get(&entity)
            .and_then(|h| self.colliders.get(*h))
            .and_then(|co| co.parent())
            .and_then(|h| self.rigid_body_entity(h))
    }

    /// Retrieve the Bevy entity the given Rapier collider (identified by its handle) is attached.
    pub fn collider_entity(&self, handle: ColliderHandle) -> Option<Entity> {
        self.colliders
            .get(handle)
            .map(|c| Entity::from_bits(c.user_data as u64))
    }

    /// Retrieve the Bevy entity the given Rapier rigid-body (identified by its handle) is attached.
    pub fn rigid_body_entity(&self, handle: RigidBodyHandle) -> Option<Entity> {
        self.bodies
            .get(handle)
            .map(|c| Entity::from_bits(c.user_data as u64))
    }

    /// Advance the simulation, based on the given timestep mode.
    #[allow(clippy::too_many_arguments)]
    pub fn step_simulation(
        &mut self,
        gravity: Vect,
        timestep_mode: TimestepMode,
        events: Option<EventWriter<CollisionEvent>>,
        hooks: &dyn PhysicsHooks,
        time: &Time,
        sim_to_render_time: &mut SimulationToRenderTime,
        mut interpolation_query: Option<
            Query<(&RapierRigidBodyHandle, &mut TransformInterpolation)>,
        >,
    ) {
        let event_queue = events.map(|e| EventQueue {
            deleted_colliders: &self.deleted_colliders,
            events: RwLock::new(e),
        });

        let events = self
            .event_handler
            .as_deref()
            .or_else(|| event_queue.as_ref().map(|q| q as &dyn EventHandler))
            .unwrap_or(&() as &dyn EventHandler);

        match timestep_mode {
            TimestepMode::Interpolated {
                dt,
                time_scale,
                substeps,
            } => {
                sim_to_render_time.diff += time.delta_seconds();

                while sim_to_render_time.diff > 0.0 {
                    // NOTE: in this comparison we do the same computations we
                    // will do for the next `while` iteration test, to make sure we
                    // don't get bit by potential float inaccuracy.
                    if sim_to_render_time.diff - dt <= 0.0 {
                        if let Some(interpolation_query) = interpolation_query.as_mut() {
                            // This is the last simulation step to be executed in the loop
                            // Update the previous state transforms
                            for (handle, mut interpolation) in interpolation_query.iter_mut() {
                                if let Some(body) = self.bodies.get(handle.0) {
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
                            &(gravity / self.physics_scale).into(),
                            &substep_integration_parameters,
                            &mut self.islands,
                            &mut self.broad_phase,
                            &mut self.narrow_phase,
                            &mut self.bodies,
                            &mut self.colliders,
                            &mut self.impulse_joints,
                            &mut self.multibody_joints,
                            &mut self.ccd_solver,
                            hooks,
                            events,
                        );
                    }

                    sim_to_render_time.diff -= dt;
                }
            }
            TimestepMode::Variable {
                max_dt,
                time_scale,
                substeps,
            } => {
                let mut substep_integration_parameters = self.integration_parameters;
                substep_integration_parameters.dt =
                    (time.delta_seconds() * time_scale).min(max_dt) / (substeps as Real);

                for _ in 0..substeps {
                    self.pipeline.step(
                        &(gravity / self.physics_scale).into(),
                        &substep_integration_parameters,
                        &mut self.islands,
                        &mut self.broad_phase,
                        &mut self.narrow_phase,
                        &mut self.bodies,
                        &mut self.colliders,
                        &mut self.impulse_joints,
                        &mut self.multibody_joints,
                        &mut self.ccd_solver,
                        hooks,
                        events,
                    );
                }
            }
            TimestepMode::Fixed { dt, substeps } => {
                let mut substep_integration_parameters = self.integration_parameters;
                substep_integration_parameters.dt = dt / (substeps as Real);

                for _ in 0..substeps {
                    self.pipeline.step(
                        &(gravity / self.physics_scale).into(),
                        &substep_integration_parameters,
                        &mut self.islands,
                        &mut self.broad_phase,
                        &mut self.narrow_phase,
                        &mut self.bodies,
                        &mut self.colliders,
                        &mut self.impulse_joints,
                        &mut self.multibody_joints,
                        &mut self.ccd_solver,
                        hooks,
                        events,
                    );
                }
            }
        }
    }

    /// This method makes sure tha the rigid-body positions have been propagated to
    /// their attached colliders, without having to perform a srimulation step.
    pub fn propagate_modified_body_positions_to_colliders(&mut self) {
        self.bodies
            .propagate_modified_body_positions_to_colliders(&mut self.colliders);
    }

    /// Updates the state of the query pipeline, based on the collider positions known
    /// from the last timestep or the last call to `self.propagate_modified_body_positions_to_colliders()`.
    pub fn update_query_pipeline(&mut self) {
        self.query_pipeline
            .update(&self.islands, &self.bodies, &self.colliders);
    }

    /// The map from entities to rigid-body handles.
    pub fn entity2body(&self) -> &HashMap<Entity, RigidBodyHandle> {
        &self.entity2body
    }

    /// The map from entities to collider handles.
    pub fn entity2collider(&self) -> &HashMap<Entity, ColliderHandle> {
        &self.entity2collider
    }

    /// The map from entities to impulse joint handles.
    pub fn entity2impulse_joint(&self) -> &HashMap<Entity, ImpulseJointHandle> {
        &self.entity2impulse_joint
    }

    /// The map from entities to multibody joint handles.
    pub fn entity2multibody_joint(&self) -> &HashMap<Entity, MultibodyJointHandle> {
        &self.entity2multibody_joint
    }

    /// Find the closest intersection between a ray and a set of collider.
    ///
    /// # Parameters
    /// * `ray_origin`: the starting point of the ray to cast.
    /// * `ray_dir`: the direction of the ray to cast.
    /// * `max_toi`: the maximum time-of-impact that can be reported by this cast. This effectively
    ///   limits the length of the ray to `ray.dir.norm() * max_toi`. Use `Real::MAX` for an unbounded ray.
    /// * `solid`: if this is `true` an impact at time 0.0 (i.e. at the ray origin) is returned if
    ///            it starts inside of a shape. If this `false` then the ray will hit the shape's boundary
    ///            even if its starts inside of it.
    /// * `query_groups`: the interaction groups which will be tested against the collider's `contact_group`
    ///                   to determine if it should be taken into account by this query.
    /// * `filter`: a more fine-grained filter. A collider is taken into account by this query if
    ///             its `contact_group` is compatible with the `query_groups`, and if this `filter`
    ///             is either `None` or returns `true`.
    pub fn cast_ray(
        &self,
        ray_origin: Vect,
        ray_dir: Vect,
        max_toi: Real,
        solid: bool,
        query_groups: InteractionGroups,
        filter: Option<&dyn Fn(Entity) -> bool>,
    ) -> Option<(Entity, Real)> {
        let ray = Ray::new(
            (ray_origin / self.physics_scale).into(),
            (ray_dir / self.physics_scale).into(),
        );
        let (h, toi) = if let Some(filter) = filter {
            self.query_pipeline.cast_ray(
                &self.colliders,
                &ray,
                max_toi,
                solid,
                query_groups,
                Some(&|h| self.collider_entity(h).map(filter).unwrap_or(false)),
            )?
        } else {
            self.query_pipeline.cast_ray(
                &self.colliders,
                &ray,
                max_toi,
                solid,
                query_groups,
                None,
            )?
        };

        self.collider_entity(h).map(|e| (e, toi))
    }

    /// Find the closest intersection between a ray and a set of collider.
    ///
    /// # Parameters
    /// * `ray_origin`: the starting point of the ray to cast.
    /// * `ray_dir`: the direction of the ray to cast.
    /// * `max_toi`: the maximum time-of-impact that can be reported by this cast. This effectively
    ///   limits the length of the ray to `ray.dir.norm() * max_toi`. Use `Real::MAX` for an unbounded ray.
    /// * `solid`: if this is `true` an impact at time 0.0 (i.e. at the ray origin) is returned if
    ///            it starts inside of a shape. If this `false` then the ray will hit the shape's boundary
    ///            even if its starts inside of it.
    /// * `query_groups`: the interaction groups which will be tested against the collider's `contact_group`
    ///                   to determine if it should be taken into account by this query.
    /// * `filter`: a more fine-grained filter. A collider is taken into account by this query if
    ///             its `contact_group` is compatible with the `query_groups`, and if this `filter`
    ///             is either `None` or returns `true`.
    pub fn cast_ray_and_get_normal(
        &self,
        ray_origin: Vect,
        ray_dir: Vect,
        max_toi: Real,
        solid: bool,
        query_groups: InteractionGroups,
        filter: Option<&dyn Fn(Entity) -> bool>,
    ) -> Option<(Entity, RayIntersection)> {
        let ray = Ray::new(
            (ray_origin / self.physics_scale).into(),
            (ray_dir / self.physics_scale).into(),
        );
        let (h, result) = if let Some(filter) = filter {
            self.query_pipeline.cast_ray_and_get_normal(
                &self.colliders,
                &ray,
                max_toi,
                solid,
                query_groups,
                Some(&|h| self.collider_entity(h).map(filter).unwrap_or(false)),
            )?
        } else {
            self.query_pipeline.cast_ray_and_get_normal(
                &self.colliders,
                &ray,
                max_toi,
                solid,
                query_groups,
                None,
            )?
        };

        self.collider_entity(h)
            .map(|e| (e, RayIntersection::from_rapier(result, ray_origin, ray_dir)))
    }

    /// Find the all intersections between a ray and a set of collider and passes them to a callback.
    ///
    /// # Parameters
    /// * `ray_origin`: the starting point of the ray to cast.
    /// * `ray_dir`: the direction of the ray to cast.
    /// * `max_toi`: the maximum time-of-impact that can be reported by this cast. This effectively
    ///   limits the length of the ray to `ray.dir.norm() * max_toi`. Use `Real::MAX` for an unbounded ray.
    /// * `solid`: if this is `true` an impact at time 0.0 (i.e. at the ray origin) is returned if
    ///            it starts inside of a shape. If this `false` then the ray will hit the shape's boundary
    ///            even if its starts inside of it.
    /// * `query_groups`: the interaction groups which will be tested against the collider's `contact_group`
    ///                   to determine if it should be taken into account by this query.
    /// * `filter`: a more fine-grained filter. A collider is taken into account by this query if
    ///             its `contact_group` is compatible with the `query_groups`, and if this `filter`
    ///             is either `None` or returns `true`.
    /// * `callback`: function executed on each collider for which a ray intersection has been found.
    ///               There is no guarantees on the order the results will be yielded. If this callback returns `false`,
    ///               this method will exit early, ignore any further raycast.
    #[allow(clippy::too_many_arguments)]
    pub fn intersections_with_ray(
        &self,
        ray_origin: Vect,
        ray_dir: Vect,
        max_toi: Real,
        solid: bool,
        query_groups: InteractionGroups,
        filter: Option<&dyn Fn(Entity) -> bool>,
        mut callback: impl FnMut(Entity, RayIntersection) -> bool,
    ) {
        let ray = Ray::new(
            (ray_origin / self.physics_scale).into(),
            (ray_dir / self.physics_scale).into(),
        );
        let callback = |h, inter: rapier::prelude::RayIntersection| {
            self.collider_entity(h)
                .map(|e| callback(e, RayIntersection::from_rapier(inter, ray_origin, ray_dir)))
                .unwrap_or(true)
        };

        if let Some(filter) = filter {
            self.query_pipeline.intersections_with_ray(
                &self.colliders,
                &ray,
                max_toi,
                solid,
                query_groups,
                Some(&|h| self.collider_entity(h).map(filter).unwrap_or(false)),
                callback,
            );
        } else {
            self.query_pipeline.intersections_with_ray(
                &self.colliders,
                &ray,
                max_toi,
                solid,
                query_groups,
                None,
                callback,
            );
        }
    }

    /// Gets the handle of up to one collider intersecting the given shape.
    ///
    /// # Parameters
    /// * `shape_pos` - The position of the shape used for the intersection test.
    /// * `shape` - The shape used for the intersection test.
    /// * `query_groups` - the interaction groups which will be tested against the collider's `contact_group`
    ///                   to determine if it should be taken into account by this query.
    /// * `filter` - a more fine-grained filter. A collider is taken into account by this query if
    ///             its `contact_group` is compatible with the `query_groups`, and if this `filter`
    ///             is either `None` or returns `true`.
    pub fn intersection_with_shape(
        &self,
        shape_pos: Vect,
        shape_rot: Rot,
        shape: &Collider,
        query_groups: InteractionGroups,
        filter: Option<&dyn Fn(Entity) -> bool>,
    ) -> Option<Entity> {
        let scaled_transform = (shape_pos / self.physics_scale, shape_rot).into();
        let mut scaled_shape = shape.clone();
        // TODO: how to set a good number of subdivisions, we don’t have access to the
        //       RapierConfiguration::scaled_shape_subdivision here.
        scaled_shape.set_scale(shape.scale / self.physics_scale, 20);

        let h = if let Some(filter) = filter {
            self.query_pipeline.intersection_with_shape(
                &self.colliders,
                &scaled_transform,
                &*scaled_shape.raw,
                query_groups,
                Some(&|h| self.collider_entity(h).map(filter).unwrap_or(false)),
            )?
        } else {
            self.query_pipeline.intersection_with_shape(
                &self.colliders,
                &scaled_transform,
                &*scaled_shape.raw,
                query_groups,
                None,
            )?
        };

        self.collider_entity(h)
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
    /// * `query_groups` - the interaction groups which will be tested against the collider's `contact_group`
    ///                   to determine if it should be taken into account by this query.
    /// * `filter` - a more fine-grained filter. A collider is taken into account by this query if
    ///             its `contact_group` is compatible with the `query_groups`, and if this `filter`
    ///             is either `None` or returns `true`.
    pub fn project_point(
        &self,
        point: Vect,
        solid: bool,
        query_groups: InteractionGroups,
        filter: Option<&dyn Fn(Entity) -> bool>,
    ) -> Option<(Entity, PointProjection)> {
        let (h, result) = if let Some(filter) = filter {
            self.query_pipeline.project_point(
                &self.colliders,
                &(point / self.physics_scale).into(),
                solid,
                query_groups,
                Some(&|h| self.collider_entity(h).map(filter).unwrap_or(false)),
            )?
        } else {
            self.query_pipeline.project_point(
                &self.colliders,
                &(point / self.physics_scale).into(),
                solid,
                query_groups,
                None,
            )?
        };

        self.collider_entity(h)
            .map(|e| (e, PointProjection::from_rapier(self.physics_scale, result)))
    }

    /// Find all the colliders containing the given point.
    ///
    /// # Parameters
    /// * `point` - The point used for the containment test.
    /// * `query_groups` - the interaction groups which will be tested against the collider's `contact_group`
    ///                   to determine if it should be taken into account by this query.
    /// * `filter` - a more fine-grained filter. A collider is taken into account by this query if
    ///             its `contact_group` is compatible with the `query_groups`, and if this `filter`
    ///             is either `None` or returns `true`.
    /// * `callback` - A function called with each collider with a shape containing the `point`.
    ///                If this callback returns `false`, this method will exit early, ignore any
    ///                further point projection.
    pub fn intersections_with_point(
        &self,
        point: Vect,
        query_groups: InteractionGroups,
        filter: Option<&dyn Fn(Entity) -> bool>,
        mut callback: impl FnMut(Entity) -> bool,
    ) {
        #[allow(clippy::redundant_closure)]
        // False-positive, we can't move callback, closure becomes `FnOnce`
        let callback = |h| self.collider_entity(h).map(|e| callback(e)).unwrap_or(true);

        if let Some(filter) = filter {
            self.query_pipeline.intersections_with_point(
                &self.colliders,
                &(point / self.physics_scale).into(),
                query_groups,
                Some(&|h| self.collider_entity(h).map(filter).unwrap_or(false)),
                callback,
            );
        } else {
            self.query_pipeline.intersections_with_point(
                &self.colliders,
                &(point / self.physics_scale).into(),
                query_groups,
                None,
                callback,
            );
        }
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
    /// * `query_groups` - the interaction groups which will be tested against the collider's `contact_group`
    ///                   to determine if it should be taken into account by this query.
    /// * `filter` - a more fine-grained filter. A collider is taken into account by this query if
    ///             its `contact_group` is compatible with the `query_groups`, and if this `filter`
    ///             is either `None` or returns `true`.
    pub fn project_point_and_get_feature(
        &self,
        point: Vect,
        // FIXME: should be a CollisionGroups
        query_groups: InteractionGroups,
        filter: Option<&dyn Fn(Entity) -> bool>,
    ) -> Option<(Entity, PointProjection, FeatureId)> {
        let (h, proj, fid) = if let Some(filter) = filter {
            self.query_pipeline.project_point_and_get_feature(
                &self.colliders,
                &(point / self.physics_scale).into(),
                query_groups,
                Some(&|h| self.collider_entity(h).map(filter).unwrap_or(false)),
            )?
        } else {
            self.query_pipeline.project_point_and_get_feature(
                &self.colliders,
                &(point / self.physics_scale).into(),
                query_groups,
                None,
            )?
        };

        self.collider_entity(h).map(|e| {
            (
                e,
                PointProjection::from_rapier(self.physics_scale, proj),
                fid,
            )
        })
    }

    /// Finds all entities of all the colliders with an AABB intersecting the given AABB.
    pub fn colliders_with_aabb_intersecting_aabb(
        &self,
        aabb: Aabb,
        mut callback: impl FnMut(Entity) -> bool,
    ) {
        #[cfg(feature = "dim2")]
        let scaled_aabb = AABB {
            mins: (aabb.min().xy() / self.physics_scale).into(),
            maxs: (aabb.max().xy() / self.physics_scale).into(),
        };
        #[cfg(feature = "dim3")]
        let scaled_aabb = AABB {
            mins: (aabb.min() / self.physics_scale).into(),
            maxs: (aabb.max() / self.physics_scale).into(),
        };
        #[allow(clippy::redundant_closure)]
        // False-positive, we can't move callback, closure becomes `FnOnce`
        let callback = |h: &ColliderHandle| {
            self.collider_entity(*h)
                .map(|e| callback(e))
                .unwrap_or(true)
        };
        self.query_pipeline
            .colliders_with_aabb_intersecting_aabb(&scaled_aabb, callback);
    }

    /// Casts a shape at a constant linear velocity and retrieve the first collider it hits.
    ///
    /// This is similar to ray-casting except that we are casting a whole shape instead of just a
    /// point (the ray origin). In the resulting `TOI`, witness and normal 1 refer to the world
    /// collider, and are in world space.
    ///
    /// # Parameters
    /// * `shape_pos` - The initial position of the shape to cast.
    /// * `shape_vel` - The constant velocity of the shape to cast (i.e. the cast direction).
    /// * `shape` - The shape to cast.
    /// * `max_toi` - The maximum time-of-impact that can be reported by this cast. This effectively
    ///   limits the distance traveled by the shape to `shapeVel.norm() * maxToi`.
    /// * `query_groups` - the interaction groups which will be tested against the collider's `contact_group`
    ///                   to determine if it should be taken into account by this query.
    /// * `filter` - a more fine-grained filter. A collider is taken into account by this query if
    ///             its `contact_group` is compatible with the `query_groups`, and if this `filter`
    ///             is either `None` or returns `true`.
    #[allow(clippy::too_many_arguments)]
    pub fn cast_shape(
        &self,
        shape_pos: Vect,
        shape_rot: Rot,
        shape_vel: Vect,
        shape: &Collider,
        max_toi: Real,
        query_groups: InteractionGroups,
        filter: Option<&dyn Fn(Entity) -> bool>,
    ) -> Option<(Entity, Toi)> {
        let scaled_transform = (shape_pos / self.physics_scale, shape_rot).into();
        let mut scaled_shape = shape.clone();
        // TODO: how to set a good number of subdivisions, we don’t have access to the
        //       RapierConfiguration::scaled_shape_subdivision here.
        scaled_shape.set_scale(shape.scale / self.physics_scale, 20);

        let (h, result) = if let Some(filter) = filter {
            self.query_pipeline.cast_shape(
                &self.colliders,
                &scaled_transform,
                &(shape_vel / self.physics_scale).into(),
                &*scaled_shape.raw,
                max_toi,
                query_groups,
                Some(&|h| self.collider_entity(h).map(filter).unwrap_or(false)),
            )?
        } else {
            self.query_pipeline.cast_shape(
                &self.colliders,
                &scaled_transform,
                &(shape_vel / self.physics_scale).into(),
                &*scaled_shape.raw,
                max_toi,
                query_groups,
                None,
            )?
        };

        self.collider_entity(h)
            .map(|e| (e, Toi::from_rapier(self.physics_scale, result)))
    }

    /* TODO: we need to wrap the NonlinearRigidMotion somehow.
     *
    /// Casts a shape with an arbitrary continuous motion and retrieve the first collider it hits.
    ///
    /// In the resulting `TOI`, witness and normal 1 refer to the world collider, and are in world
    /// space.
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
    /// * `query_groups` - the interaction groups which will be tested against the collider's `contact_group`
    ///                   to determine if it should be taken into account by this query.
    /// * `filter` - a more fine-grained filter. A collider is taken into account by this query if
    ///             its `contact_group` is compatible with the `query_groups`, and if this `filter`
    ///             is either `None` or returns `true`.
    pub fn nonlinear_cast_shape(
        &self,
        shape_motion: &NonlinearRigidMotion,
        shape: &Collider,
        start_time: Real,
        end_time: Real,
        stop_at_penetration: bool,
        query_groups: InteractionGroups,
        filter: Option<&dyn Fn(Entity) -> bool>,
    ) -> Option<(Entity, Toi)> {
        let scaled_transform = (shape_pos * self.physics_scale, shape_rot).into();
        let mut scaled_shape = shape.clone();
        // TODO: how to set a good number of subdivisions, we don’t have access to the
        //       RapierConfiguration::scaled_shape_subdivision here.
        scaled_shape.set_scale(shape.scale * self.physics_scale, 20);

        let (h, result) = if let Some(filter) = filter {
            self.query_pipeline.nonlinear_cast_shape(
                &self.colliders,
                shape_motion,
                &*scaled_shape.raw,
                start_time,
                end_time,
                stop_at_penetration,
                query_groups,
                Some(&|h| self.collider_entity(h).map(filter).unwrap_or(false)),
            )?
        } else {
            self.query_pipeline.nonlinear_cast_shape(
                &self.colliders,
                shape_motion,
                &*scaled_shape.raw,
                start_time,
                end_time,
                stop_at_penetration,
                query_groups,
                None,
            )?
        };

        self.collider_entity(h).map(|e| (e, result))
    }
     */

    /// Retrieve all the colliders intersecting the given shape.
    ///
    /// # Parameters
    /// * `shapePos` - The position of the shape to test.
    /// * `shapeRot` - The orientation of the shape to test.
    /// * `shape` - The shape to test.
    /// * `query_groups` - the interaction groups which will be tested against the collider's `contact_group`
    ///                   to determine if it should be taken into account by this query.
    /// * `filter` - a more fine-grained filter. A collider is taken into account by this query if
    ///             its `contact_group` is compatible with the `query_groups`, and if this `filter`
    ///             is either `None` or returns `true`.
    /// * `callback` - A function called with the entities of each collider intersecting the `shape`.
    pub fn intersections_with_shape(
        &self,
        shape_pos: Vect,
        shape_rot: Rot,
        shape: &Collider,
        query_groups: InteractionGroups,
        filter: Option<&dyn Fn(Entity) -> bool>,
        mut callback: impl FnMut(Entity) -> bool,
    ) {
        let scaled_transform = (shape_pos / self.physics_scale, shape_rot).into();
        let mut scaled_shape = shape.clone();
        // TODO: how to set a good number of subdivisions, we don’t have access to the
        //       RapierConfiguration::scaled_shape_subdivision here.
        scaled_shape.set_scale(shape.scale / self.physics_scale, 20);

        #[allow(clippy::redundant_closure)]
        // False-positive, we can't move callback, closure becomes `FnOnce`
        let callback = |h| self.collider_entity(h).map(|e| callback(e)).unwrap_or(true);

        if let Some(filter) = filter {
            self.query_pipeline.intersections_with_shape(
                &self.colliders,
                &scaled_transform,
                &*scaled_shape.raw,
                query_groups,
                Some(&|h| self.collider_entity(h).map(filter).unwrap_or(false)),
                callback,
            );
        } else {
            self.query_pipeline.intersections_with_shape(
                &self.colliders,
                &scaled_transform,
                &*scaled_shape.raw,
                query_groups,
                None,
                callback,
            );
        }
    }
}
