use crate::pipeline::{CollisionEvent, ContactForceEvent, PhysicsHooksWithQueryResource};
use crate::plugin::configuration::SimulationToRenderTime;
use crate::plugin::{systems, RapierConfiguration, RapierContext};
use crate::prelude::*;
use bevy::ecs::{event::Events, query::WorldQuery};
use bevy::prelude::*;
use std::marker::PhantomData;

/// No specific user-data is associated to the hooks.
pub type NoUserData = ();

/// A plugin responsible for setting up a full Rapier physics simulation pipeline and resources.
//
// This will automatically setup all the resources needed to run a physics simulation with the
// Rapier physics engine.
pub struct RapierPhysicsPlugin<PhysicsHooksData = ()> {
    physics_scale: f32,
    default_system_setup: bool,
    _phantom: PhantomData<PhysicsHooksData>,
}

impl<PhysicsHooksData: 'static + WorldQuery + Send + Sync> RapierPhysicsPlugin<PhysicsHooksData> {
    /// Specifies a scale ratio between the physics world and the bevy transforms.
    ///
    /// This affects the size of every elements in the physics engine, by multiplying
    /// all the length-related quantities by the `physics_scale` factor. This should
    /// likely always be 1.0 in 3D. In 2D, this is useful to specify a "pixels-per-meter"
    /// conversion ratio.
    pub fn with_physics_scale(mut self, physics_scale: f32) -> Self {
        self.physics_scale = physics_scale;
        self
    }

    /// Specifies whether the plugin should setup each of its [`PhysicsStages`]
    /// (`true`), or if the user will set them up later (`false`).
    ///
    /// The default value is `true`.
    pub fn with_default_system_setup(mut self, default_system_setup: bool) -> Self {
        self.default_system_setup = default_system_setup;
        self
    }

    /// Specifies how many pixels on the 2D canvas equal one meter on the physics world.
    ///
    /// This conversion unit assumes that the 2D camera uses an unscaled projection.
    #[cfg(feature = "dim2")]
    pub fn pixels_per_meter(pixels_per_meter: f32) -> Self {
        Self {
            physics_scale: pixels_per_meter,
            default_system_setup: true,
            _phantom: PhantomData,
        }
    }

    /// Provided for use when staging systems outside of this plugin using
    /// [`with_system_setup(false)`](Self::with_system_setup).
    /// See [`PhysicsSet`] for a description of these systems.
    pub fn get_systems(set: PhysicsSet) -> SystemConfigs {
        // A set for `propagate_transforms` to mark it as ambiguous with `sync_simple_transforms`.
        // Used instead of the `SystemTypeSet` as that would not allow multiple instances of the system.
        #[derive(Debug, Hash, PartialEq, Eq, Clone, SystemSet)]
        struct PropagateTransformsSet;

        match set {
            PhysicsSet::SyncBackend => (
                // Run the character controller before the manual transform propagation.
                systems::update_character_controls,
                // Run Bevy transform propagation additionally to sync [`GlobalTransform`]
                bevy::transform::systems::sync_simple_transforms
                    .in_set(RapierTransformPropagateSet)
                    .after(systems::update_character_controls),
                bevy::transform::systems::propagate_transforms
                    .after(systems::update_character_controls)
                    .in_set(PropagateTransformsSet)
                    .in_set(RapierTransformPropagateSet),
                systems::init_async_colliders.after(RapierTransformPropagateSet),
                systems::apply_scale.after(systems::init_async_colliders),
                systems::apply_collider_user_changes.after(systems::apply_scale),
                systems::apply_rigid_body_user_changes.after(systems::apply_collider_user_changes),
                systems::apply_joint_user_changes.after(systems::apply_rigid_body_user_changes),
                systems::init_rigid_bodies.after(systems::apply_joint_user_changes),
                systems::init_colliders
                    .after(systems::init_rigid_bodies)
                    .after(systems::init_async_colliders),
                systems::init_joints.after(systems::init_colliders),
                systems::apply_initial_rigid_body_impulses.after(systems::init_colliders),
                systems::sync_removals
                    .after(systems::init_joints)
                    .after(systems::apply_initial_rigid_body_impulses),
                #[cfg(all(feature = "dim3", feature = "async-collider"))]
                systems::init_async_scene_colliders
                    .after(bevy::scene::scene_spawner_system)
                    .before(systems::init_async_colliders),
            )
                .into_configs(),
            PhysicsSet::SyncBackendFlush => (apply_system_buffers,).into_configs(),
            PhysicsSet::StepSimulation => (
                systems::step_simulation::<PhysicsHooks>,
                Events::<CollisionEvent>::update_system
                    .before(systems::step_simulation::<PhysicsHooks>),
                Events::<ContactForceEvent>::update_system
                    .before(systems::step_simulation::<PhysicsHooks>),
            )
                .into_configs(),
            PhysicsSet::Writeback => (
                systems::update_colliding_entities,
                systems::writeback_rigid_bodies,
            )
                .into_configs(),
        }
    }
}

impl<PhysicsHooksData> Default for RapierPhysicsPlugin<PhysicsHooksData> {
    fn default() -> Self {
        Self {
            physics_scale: 1.0,
            default_system_setup: true,
            _phantom: PhantomData,
        }
    }
}

#[derive(Debug, Hash, PartialEq, Eq, Clone, StageLabel)]
/// [`StageLabel`] for each phase of the plugin.
pub enum PhysicsStages {
    /// This stage runs the systems responsible for synchronizing (and
    /// initializing) backend data structures with current component state.
    /// These systems typically run at the after [`CoreStage::Update`].
    SyncBackend,
    /// The systems responsible for advancing the physics simulation, and
    /// updating the internal state for scene queries.
    /// These systems typically run immediately after [`PhysicsStages::SyncBackend`].
    StepSimulation,
    /// The systems responsible for updating
    /// [`crate::geometry::collider::CollidingEntities`] and writing
    /// the result of the last simulation step into our `bevy_rapier`
    /// components and the [`GlobalTransform`] component.
    /// These systems typically run immediately after [`PhysicsStages::StepSimulation`].
    Writeback,
    /// The systems responsible for removing from Rapier the
    /// rigid-bodies/colliders/joints which had their related `bevy_rapier`
    /// components removed by the user (through component removal or despawn).
    /// These systems typically run at the start of [`CoreStage::Last`].
    DetectDespawn,
}

impl<PhysicsHooksData: 'static + WorldQuery + Send + Sync> Plugin
    for RapierPhysicsPlugin<PhysicsHooksData>
{
    fn build(&self, app: &mut App) {
        // Register components as reflectable.
        app.register_type::<RigidBody>()
            .register_type::<Velocity>()
            .register_type::<AdditionalMassProperties>()
            .register_type::<MassProperties>()
            .register_type::<LockedAxes>()
            .register_type::<ExternalForce>()
            .register_type::<ExternalImpulse>()
            .register_type::<Sleeping>()
            .register_type::<Damping>()
            .register_type::<Dominance>()
            .register_type::<Ccd>()
            .register_type::<GravityScale>()
            .register_type::<CollidingEntities>()
            .register_type::<Sensor>()
            .register_type::<Friction>()
            .register_type::<Restitution>()
            .register_type::<CollisionGroups>()
            .register_type::<SolverGroups>()
            .register_type::<ContactForceEventThreshold>()
            .register_type::<Group>();

        // Insert all of our required resources. Don’t overwrite
        // the `RapierConfiguration` if it already exists.
        if app.world.get_resource::<RapierConfiguration>().is_none() {
            app.insert_resource(RapierConfiguration::default());
        }

        app.insert_resource(SimulationToRenderTime::default())
            .insert_resource(RapierContext {
                physics_scale: self.physics_scale,
                ..Default::default()
            })
            .insert_resource(Events::<CollisionEvent>::default())
            .insert_resource(Events::<ContactForceEvent>::default());

        // Add each stage as necessary
        if self.default_system_setup {
            app.add_stage_after(
                CoreStage::Update,
                PhysicsStages::SyncBackend,
                SystemStage::parallel()
                    .with_system_set(Self::get_systems(PhysicsStages::SyncBackend)),
            );
            app.add_stage_after(
                PhysicsStages::SyncBackend,
                PhysicsStages::StepSimulation,
                SystemStage::parallel()
                    .with_system_set(Self::get_systems(PhysicsStages::StepSimulation)),
            );
            app.add_stage_after(
                PhysicsStages::StepSimulation,
                PhysicsStages::Writeback,
                SystemStage::parallel()
                    .with_system_set(Self::get_systems(PhysicsStages::Writeback)),
            );

            // NOTE: we run sync_removals at the end of the frame, too, in order to make sure we don’t miss any `RemovedComponents`.
            app.add_stage_before(
                CoreStage::Last,
                PhysicsStages::DetectDespawn,
                SystemStage::parallel()
                    .with_system_set(Self::get_systems(PhysicsStages::DetectDespawn)),
            );
        }

        if app
            .world
            .get_resource::<PhysicsHooksWithQueryResource<PhysicsHooksData>>()
            .is_none()
        {
            app.insert_resource(PhysicsHooksWithQueryResource::<PhysicsHooksData>(Box::new(
                (),
            )));
        }
    }
}
