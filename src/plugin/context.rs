use bevy::prelude::*;
use std::collections::{HashMap, HashSet};
use std::sync::RwLock;

use rapier::prelude::{
    BroadPhase, CCDSolver, ColliderHandle, ColliderSet, EventHandler, FeatureId,
    ImpulseJointHandle, ImpulseJointSet, IntegrationParameters, IslandManager,
    MultibodyJointHandle, MultibodyJointSet, NarrowPhase, PhysicsHooks, PhysicsPipeline,
    PointProjection, QueryFilter as RapierQueryFilter, QueryPipeline, Ray, RayIntersection, Real,
    RigidBodyHandle, RigidBodySet, TOI,
};

use crate::geometry::Collider;
use crate::math::{Isometry, Vect};
use crate::pipeline::{CollisionEvent, ContactForceEvent, EventQueue, QueryFilter};
use bevy::prelude::{Entity, EventWriter, GlobalTransform, Query};

use crate::control::{CharacterCollision, MoveShapeOptions, MoveShapeOutput};
use crate::dynamics::TransformInterpolation;
use crate::plugin::configuration::{SimulationToRenderTime, TimestepMode};
use rapier::control::{CharacterAutostep, EffectiveCharacterMovement};

/// The Rapier context, containing all the state of the physics engine.
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Resource)]
pub struct RapierContext {
    /// The island manager, which detects what object is sleeping
    /// (not moving much) to reduce computations.
    pub islands: IslandManager,
    /// The broad-phase, which detects potential contact pairs.
    pub broad_phase: BroadPhase,
    /// The narrow-phase, which computes contact points, tests intersections,
    /// and maintains the contact and intersection graphs.
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
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub(crate) event_handler: Option<Box<dyn EventHandler>>,
    // For transform change detection.
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub(crate) last_body_transform_set: HashMap<RigidBodyHandle, GlobalTransform>,
    // This maps the handles of colliders that have been deleted since the last
    // physics update, to the entity they were attached to.
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub(crate) deleted_colliders: HashSet<Entity>, // TODO: still needed?
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub(crate) character_collisions_collector: Vec<rapier::control::CharacterCollision>,
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
            event_handler: None,
            last_body_transform_set: HashMap::new(),
            deleted_colliders: HashSet::new(),
            character_collisions_collector: vec![],
        }
    }
}

impl RapierContext {
    /// If the collider attached to `entity` is attached to a rigid-body, this
    /// returns the `Entity` containing that rigid-body.
    pub fn collider_parent(&self, entity: Entity) -> Option<Entity> {
        self.colliders.get(entity).and_then(|co| co.parent())
    }

    /// If entity is a rigid-body, this returns the collider `Entity`s attached
    /// to that rigid-body.
    pub fn rigid_body_colliders(&self, entity: Entity) -> impl Iterator<Item = Entity> + '_ {
        self.bodies
            .get(entity)
            .into_iter()
            .flat_map(|body| body.colliders().iter().copied())
    }

    /// Advance the simulation, based on the given timestep mode.
    #[allow(clippy::too_many_arguments)]
    pub fn step_simulation(
        &mut self,
        gravity: Vect,
        timestep_mode: TimestepMode,
        events: Option<(EventWriter<CollisionEvent>, EventWriter<ContactForceEvent>)>,
        hooks: &dyn PhysicsHooks,
        time: &Time,
        sim_to_render_time: &mut SimulationToRenderTime,
        mut interpolation_query: Option<Query<(Entity, &mut TransformInterpolation)>>,
    ) {
        let event_queue = events.map(|(ce, fe)| EventQueue {
            deleted_colliders: &self.deleted_colliders,
            collision_events: RwLock::new(ce),
            contact_force_events: RwLock::new(fe),
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
                self.integration_parameters.dt = dt;

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
                                if let Some(body) = self.bodies.get(handle) {
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
                            &gravity,
                            &substep_integration_parameters,
                            &mut self.islands,
                            &mut self.broad_phase,
                            &mut self.narrow_phase,
                            &mut self.bodies,
                            &mut self.colliders,
                            &mut self.impulse_joints,
                            &mut self.multibody_joints,
                            &mut self.ccd_solver,
                            None,
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
                self.integration_parameters.dt = (time.delta_seconds() * time_scale).min(max_dt);

                let mut substep_integration_parameters = self.integration_parameters;
                substep_integration_parameters.dt /= substeps as Real;

                for _ in 0..substeps {
                    self.pipeline.step(
                        &gravity,
                        &substep_integration_parameters,
                        &mut self.islands,
                        &mut self.broad_phase,
                        &mut self.narrow_phase,
                        &mut self.bodies,
                        &mut self.colliders,
                        &mut self.impulse_joints,
                        &mut self.multibody_joints,
                        &mut self.ccd_solver,
                        None,
                        hooks,
                        events,
                    );
                }
            }
            TimestepMode::Fixed { dt, substeps } => {
                self.integration_parameters.dt = dt;

                let mut substep_integration_parameters = self.integration_parameters;
                substep_integration_parameters.dt = dt / (substeps as Real);

                for _ in 0..substeps {
                    self.pipeline.step(
                        &gravity,
                        &substep_integration_parameters,
                        &mut self.islands,
                        &mut self.broad_phase,
                        &mut self.narrow_phase,
                        &mut self.bodies,
                        &mut self.colliders,
                        &mut self.impulse_joints,
                        &mut self.multibody_joints,
                        &mut self.ccd_solver,
                        None,
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
        self.query_pipeline.update(&self.bodies, &self.colliders);
    }

    /// Attempts to move shape, optionally sliding or climbing obstacles.
    ///
    /// # Parameters
    /// * `movement`: the translational movement to apply.
    /// * `shape`: the shape to move.
    /// * `shape_translation`: the initial position of the shape.
    /// * `shape_rotation`: the rotation of the shape.
    /// * `shape_mass`: the mass of the shape to be considered by the impulse calculation if
    ///                 `MoveShapeOptions::apply_impulse_to_dynamic_bodies` is set to true.
    /// * `options`: configures the behavior of the automatic sliding and climbing.
    /// * `filter`: indicates what collider or rigid-body needs to be ignored by the obstacle detection.
    /// * `events`: callback run on each obstacle hit by the shape on its path.
    #[allow(clippy::too_many_arguments)]
    pub fn move_shape(
        &mut self,
        movement: Vect,
        shape: &Collider,
        shape_pos: Isometry,
        shape_mass: Real,
        options: &MoveShapeOptions,
        filter: QueryFilter,
        mut events: impl FnMut(CharacterCollision),
    ) -> EffectiveCharacterMovement {
        let up = options
            .up
            .try_into()
            .expect("The up vector must be non-zero.");
        let autostep = options.autostep;
        let controller = rapier::control::KinematicCharacterController {
            up,
            offset: options.offset,
            slide: options.slide,
            autostep,
            max_slope_climb_angle: options.max_slope_climb_angle,
            min_slope_slide_angle: options.min_slope_slide_angle,
            snap_to_ground: options.snap_to_ground,
        };

        self.character_collisions_collector.clear();

        // TODO: having to grab all the references to avoid having self in
        //       the closure is ugly.
        let dt = self.integration_parameters.dt;
        let colliders = &self.colliders;
        let bodies = &mut self.bodies;
        let query_pipeline = &self.query_pipeline;
        let collisions = &mut self.character_collisions_collector;
        collisions.clear();

        let result = controller.move_shape(
            dt,
            bodies,
            colliders,
            query_pipeline,
            &*shape.raw,
            &shape_pos,
            movement,
            filter,
            |c| {
                events(c);
                collisions.push(c);
            },
        );

        if options.apply_impulse_to_dynamic_bodies {
            for collision in &*collisions {
                controller.solve_character_collision_impulses(
                    dt,
                    bodies,
                    colliders,
                    query_pipeline,
                    &*shape.raw,
                    shape_mass,
                    collision,
                    filter,
                )
            }
        }

        result
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
    /// * `filter`: set of rules used to determine which collider is taken into account by this scene query.
    pub fn cast_ray(
        &self,
        ray: impl Into<Ray>,
        max_toi: Real,
        solid: bool,
        filter: QueryFilter,
    ) -> Option<(Entity, Real)> {
        self.query_pipeline.cast_ray(
            &self.bodies,
            &self.colliders,
            &ray.into(),
            max_toi,
            solid,
            filter,
        )
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
    /// * `filter`: set of rules used to determine which collider is taken into account by this scene query.
    pub fn cast_ray_and_get_normal(
        &self,
        ray: impl Into<Ray>,
        max_toi: Real,
        solid: bool,
        filter: QueryFilter,
    ) -> Option<(Entity, RayIntersection)> {
        self.query_pipeline.cast_ray_and_get_normal(
            &self.bodies,
            &self.colliders,
            &ray.into(),
            max_toi,
            solid,
            filter,
        )
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
    /// * `filter`: set of rules used to determine which collider is taken into account by this scene query.
    /// * `callback`: function executed on each collider for which a ray intersection has been found.
    ///               There is no guarantees on the order the results will be yielded. If this callback returns `false`,
    ///               this method will exit early, ignore any further raycast.
    #[allow(clippy::too_many_arguments)]
    pub fn intersections_with_ray(
        &self,
        ray: impl Into<Ray>,
        max_toi: Real,
        solid: bool,
        filter: QueryFilter,
        callback: impl FnMut(Entity, RayIntersection) -> bool,
    ) {
        self.query_pipeline.intersections_with_ray(
            &self.bodies,
            &self.colliders,
            &ray.into(),
            max_toi,
            solid,
            filter,
            callback,
        )
    }

    /// Gets the handle of up to one collider intersecting the given shape.
    ///
    /// # Parameters
    /// * `shape_pos` - The position of the shape used for the intersection test.
    /// * `shape` - The shape used for the intersection test.
    /// * `filter`: set of rules used to determine which collider is taken into account by this scene query.
    pub fn intersection_with_shape(
        &self,
        shape_pos: Isometry,
        shape: &Collider,
        filter: QueryFilter,
    ) -> Option<Entity> {
        self.query_pipeline.intersection_with_shape(
            &self.bodies,
            &self.colliders,
            &shape_pos,
            &*shape.raw,
            filter,
        )
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
    /// * `filter`: set of rules used to determine which collider is taken into account by this scene query.
    pub fn project_point(
        &self,
        point: Vect,
        solid: bool,
        filter: QueryFilter,
    ) -> Option<(Entity, PointProjection)> {
        self.query_pipeline
            .project_point(&self.bodies, &self.colliders, &point, solid, filter)
    }

    /// Find all the colliders containing the given point.
    ///
    /// # Parameters
    /// * `point` - The point used for the containment test.
    /// * `filter`: set of rules used to determine which collider is taken into account by this scene query.
    /// * `callback` - A function called with each collider with a shape containing the `point`.
    ///                If this callback returns `false`, this method will exit early, ignore any
    ///                further point projection.
    pub fn intersections_with_point(
        &self,
        point: Vect,
        filter: QueryFilter,
        mut callback: impl FnMut(Entity) -> bool,
    ) {
        self.query_pipeline.intersections_with_point(
            &self.bodies,
            &self.colliders,
            &point,
            filter,
            callback,
        )
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
    /// * `filter`: set of rules used to determine which collider is taken into account by this scene query.
    pub fn project_point_and_get_feature(
        &self,
        point: Vect,
        filter: QueryFilter,
    ) -> Option<(Entity, PointProjection, FeatureId)> {
        self.query_pipeline.project_point_and_get_feature(
            &self.bodies,
            &self.colliders,
            &point,
            filter,
        )
    }

    /// Finds all entities of all the colliders with an Aabb intersecting the given Aabb.
    #[cfg(not(feature = "headless"))]
    pub fn colliders_with_aabb_intersecting_aabb(
        &self,
        aabb: bevy::render::primitives::Aabb,
        mut callback: impl FnMut(&Entity) -> bool,
    ) {
        #[cfg(feature = "dim2")]
        let scaled_aabb = rapier::prelude::Aabb {
            mins: aabb.min().xy(),
            maxs: aabb.max().xy(),
        };
        #[cfg(feature = "dim3")]
        let scaled_aabb = rapier::prelude::Aabb {
            mins: aabb.min().into(),
            maxs: aabb.max().into(),
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
    /// * `shape_pos` - The initial translation of the shape to cast.
    /// * `shape_rot` - The rotation of the shape to cast.
    /// * `shape_vel` - The constant velocity of the shape to cast (i.e. the cast direction).
    /// * `shape` - The shape to cast.
    /// * `max_toi` - The maximum time-of-impact that can be reported by this cast. This effectively
    ///   limits the distance traveled by the shape to `shapeVel.norm() * maxToi`.
    /// * `stop_at_penetration` - If the casted shape starts in a penetration state with any
    ///    collider, two results are possible. If `stop_at_penetration` is `true` then, the
    ///    result will have a `toi` equal to `start_time`. If `stop_at_penetration` is `false`
    ///    then the nonlinear shape-casting will see if further motion wrt. the penetration normal
    ///    would result in tunnelling. If it does not (i.e. we have a separating velocity along
    ///    that normal) then the nonlinear shape-casting will attempt to find another impact,
    ///    at a time `> start_time` that could result in tunnelling.
    /// * `filter`: set of rules used to determine which collider is taken into account by this scene query.
    #[allow(clippy::too_many_arguments)]
    pub fn cast_shape(
        &self,
        shape_pos: Isometry,
        shape_vel: Vect,
        shape: &Collider,
        max_toi: Real,
        stop_at_penetration: bool,
        filter: QueryFilter,
    ) -> Option<(Entity, TOI)> {
        self.query_pipeline.cast_shape(
            &self.bodies,
            &self.colliders,
            &shape_pos,
            &shape_vel,
            &*shape.raw,
            max_toi,
            stop_at_penetration,
            filter,
        )
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
    /// * `filter`: set of rules used to determine which collider is taken into account by this scene query.
    pub fn nonlinear_cast_shape(
        &self,
        shape_motion: &NonlinearRigidMotion,
        shape: &Collider,
        start_time: Real,
        end_time: Real,
        stop_at_penetration: bool,
        filter: QueryFilter,
    ) -> Option<(Entity, TOI)> {
        let scaled_transform = (shape_pos, shape_rot).into();
        let mut scaled_shape = shape.clone();
        // TODO: how to set a good number of subdivisions, we don’t have access to the
        //       RapierConfiguration::scaled_shape_subdivision here.
        scaled_shape.set_scale(shape.scale, 20);

        let (h, result) = self.with_query_filter(filter, move |filter| {
            self.query_pipeline.nonlinear_cast_shape(
                &self.bodies,
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

    /// Retrieve all the colliders intersecting the given shape.
    ///
    /// # Parameters
    /// * `shapePos` - The position of the shape to test.
    /// * `shapeRot` - The orientation of the shape to test.
    /// * `shape` - The shape to test.
    /// * `filter`: set of rules used to determine which collider is taken into account by this scene query.
    /// * `callback` - A function called with the entities of each collider intersecting the `shape`.
    pub fn intersections_with_shape(
        &self,
        shape_pos: Isometry,
        shape: &Collider,
        filter: QueryFilter,
        callback: impl FnMut(Entity) -> bool,
    ) {
        self.query_pipeline.intersections_with_shape(
            &self.bodies,
            &self.colliders,
            &shape_pos,
            &*shape.raw,
            filter,
            callback,
        )
    }
}
