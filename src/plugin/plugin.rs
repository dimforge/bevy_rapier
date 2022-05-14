use crate::pipeline::{CollisionEvent, PhysicsHooksWithQueryResource};
use crate::plugin::configuration::SimulationToRenderTime;
use crate::plugin::{systems, RapierConfiguration, RapierContext};
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
    system_setup: bool,
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
    pub fn with_system_setup(mut self, system_setup: bool) -> Self {
        self.system_setup = system_setup;
        self
    }

    /// Specifies how many pixels on the 2D canvas equal one meter on the physics world.
    ///
    /// This conversion unit assumes that the 2D camera uses an unscaled projection.
    #[cfg(feature = "dim2")]
    pub fn pixels_per_meter(pixels_per_meter: f32) -> Self {
        Self {
            physics_scale: pixels_per_meter,
            system_setup: true,
            _phantom: PhantomData,
        }
    }

    /// Provided for use when staging systems outside of this plugin using
    /// [`with_system_setup(false)`](Self::with_system_setup).
    /// See [`PhysicsStages::SyncBackend`] for a description of these systems.
    pub fn get_sync_backend_systems() -> SystemSet {
        SystemSet::new()
            .with_system(systems::init_async_colliders)
            .with_system(systems::apply_scale.after(systems::init_async_colliders))
            .with_system(systems::apply_collider_user_changes.after(systems::apply_scale))
            .with_system(
                systems::apply_rigid_body_user_changes.after(systems::apply_collider_user_changes),
            )
            .with_system(
                systems::apply_joint_user_changes.after(systems::apply_rigid_body_user_changes),
            )
            .with_system(systems::init_rigid_bodies.after(systems::apply_joint_user_changes))
            .with_system(
                systems::init_colliders
                    .after(systems::init_rigid_bodies)
                    .after(systems::init_async_colliders),
            )
            .with_system(systems::init_joints.after(systems::init_colliders))
            .with_system(systems::sync_removals.after(systems::init_joints))
    }

    /// Provided for use when staging systems outside of this plugin using
    /// [`with_system_setup(false)`](Self::with_system_setup).
    /// See [`PhysicsStages::StepSimulation`] for a description of these systems.
    pub fn get_step_simulation_systems() -> SystemSet {
        SystemSet::new().with_system(systems::step_simulation::<PhysicsHooksData>)
    }

    /// Provided for use when staging systems outside of this plugin using
    /// [`with_system_setup(false)`](Self::with_system_setup).
    /// See [`PhysicsStages::Writeback`] for a description of these systems.
    pub fn get_writeback_systems() -> SystemSet {
        SystemSet::new()
            .with_system(systems::update_colliding_entities)
            .with_system(systems::writeback_rigid_bodies)
    }

    /// Provided for use when staging systems outside of this plugin using
    /// [`with_system_setup(false)`](Self::with_system_setup).
    /// See [`PhysicsStages::DetectDespawn`] for a description of these systems.
    pub fn get_detect_despawn_systems() -> SystemSet {
        SystemSet::new().with_system(systems::sync_removals)
    }
}

impl<PhysicsHooksData> Default for RapierPhysicsPlugin<PhysicsHooksData> {
    fn default() -> Self {
        Self {
            physics_scale: 1.0,
            system_setup: true,
            _phantom: PhantomData,
        }
    }
}

#[derive(Debug, Hash, PartialEq, Eq, Clone, StageLabel)]
/// [`StageLabel`] for each phase of the plugin.
pub enum PhysicsStages {
    /// This stage runs the systems responsible for synchronizing (and
    /// initializing) backend data structures with current component state.
    /// These systems typically run at the start of [`CoreStage::Update`].
    SyncBackend,
    /// The systems responsible for advancing the physics simulation, and
    /// updating the internal state for scene queries.
    /// These systems typically run immediately after [`PhysicsStages::SyncBackend`].
    StepSimulation,
    /// The systems responsible for updating
    /// [`crate::geometry::collider::CollidingEntities`] and writing
    /// the result of the last simulation step into our `bevy_rapier`
    /// components and the [`Transform`] component.
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
        // Insert all of our required resources
        app.insert_resource(RapierConfiguration::default())
            .insert_resource(SimulationToRenderTime::default())
            .insert_resource(RapierContext {
                physics_scale: self.physics_scale,
                ..Default::default()
            })
            .insert_resource(Events::<CollisionEvent>::default());

        // Add each stage as necessary
        if self.system_setup {
            app.add_stage_before(
                CoreStage::Update,
                PhysicsStages::SyncBackend,
                SystemStage::parallel().with_system_set(Self::get_sync_backend_systems()),
            );
            app.add_stage_after(
                PhysicsStages::SyncBackend,
                PhysicsStages::StepSimulation,
                SystemStage::parallel().with_system_set(Self::get_step_simulation_systems()),
            );
            app.add_stage_after(
                PhysicsStages::StepSimulation,
                PhysicsStages::Writeback,
                SystemStage::parallel().with_system_set(Self::get_writeback_systems()),
            );

            // NOTE: we run sync_removals at the end of the frame, too, in order to make sure we don’t miss any `RemovedComponents`.
            app.add_stage_before(
                CoreStage::Last,
                PhysicsStages::DetectDespawn,
                SystemStage::parallel().with_system_set(Self::get_detect_despawn_systems()),
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
