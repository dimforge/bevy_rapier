use crate::pipeline::{CollisionEvent, ContactForceEvent};
use crate::plugin::configuration::SimulationToRenderTime;
use crate::plugin::{systems, RapierConfiguration, RapierContext};
use crate::prelude::*;
use bevy::ecs::{
    intern::Interned,
    schedule::{ScheduleLabel, SystemConfigs},
    system::SystemParamItem,
};
use bevy::{prelude::*, transform::TransformSystem};
use std::marker::PhantomData;

use super::context::DefaultRapierContext;

/// No specific user-data is associated to the hooks.
pub type NoUserData = ();

/// A plugin responsible for setting up a full Rapier physics simulation pipeline and resources.
///
/// This will automatically setup all the resources needed to run a physics simulation with the
/// Rapier physics engine.
pub struct RapierPhysicsPlugin<PhysicsHooks = ()> {
    schedule: Interned<dyn ScheduleLabel>,
    default_system_setup: bool,
    /// Read in PreStartup, this will create a default world.
    pub default_world_setup: Option<RapierContextInitialization>,
    _phantom: PhantomData<PhysicsHooks>,
}

impl<PhysicsHooks> RapierPhysicsPlugin<PhysicsHooks>
where
    PhysicsHooks: 'static + BevyPhysicsHooks,
    for<'w, 's> SystemParamItem<'w, 's, PhysicsHooks>: BevyPhysicsHooks,
{
    /// Specifies a scale ratio between the physics world and the bevy transforms.
    ///
    /// This affects the size of every elements in the physics engine, by multiplying
    /// all the length-related quantities by the `length_unit` factor. This should
    /// likely always be 1.0 in 3D. In 2D, this is useful to specify a "pixels-per-meter"
    /// conversion ratio.
    pub fn with_length_unit(mut self, length_unit: f32) -> Self {
        self.default_world_setup = Some(RapierContextInitialization {
            length_unit: length_unit,
        });
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
            default_system_setup: true,
            default_world_setup: Some(RapierContextInitialization {
                length_unit: pixels_per_meter,
            }),
            ..default()
        }
    }

    /// Adds the physics systems to the `FixedUpdate` schedule rather than `PostUpdate`.
    pub fn in_fixed_schedule(self) -> Self {
        self.in_schedule(FixedUpdate)
    }

    /// Adds the physics systems to the provided schedule rather than `PostUpdate`.
    pub fn in_schedule(mut self, schedule: impl ScheduleLabel) -> Self {
        self.schedule = schedule.intern();
        self
    }

    /// Provided for use when staging systems outside of this plugin using
    /// [`with_system_setup(false)`](Self::with_system_setup).
    /// See [`PhysicsSet`] for a description of these systems.
    pub fn get_systems(set: PhysicsSet) -> SystemConfigs {
        match set {
            PhysicsSet::SyncBackend => (
                // Run the character controller before the manual transform propagation.
                systems::update_character_controls,
                // Run Bevy transform propagation additionally to sync [`GlobalTransform`]
                (
                    bevy::transform::systems::sync_simple_transforms,
                    bevy::transform::systems::propagate_transforms,
                )
                    .chain()
                    .in_set(RapierTransformPropagateSet),
                #[cfg(all(feature = "dim3", feature = "async-collider"))]
                systems::init_async_scene_colliders,
                #[cfg(all(feature = "dim3", feature = "async-collider"))]
                systems::init_async_colliders,
                systems::init_rigid_bodies,
                systems::init_colliders,
                systems::init_joints,
                systems::sync_removals,
                // Run this here so the following systems do not have a 1 frame delay.
                apply_deferred,
                systems::apply_scale,
                systems::apply_collider_user_changes,
                systems::apply_rigid_body_user_changes,
                systems::apply_joint_user_changes,
                systems::apply_initial_rigid_body_impulses,
            )
                .chain()
                .into_configs(),
            PhysicsSet::StepSimulation => (systems::step_simulation::<PhysicsHooks>).into_configs(),
            PhysicsSet::Writeback => (
                systems::update_colliding_entities,
                systems::writeback_rigid_bodies,
                systems::writeback_mass_properties,
            )
                .into_configs(),
        }
    }
}

/// A set for rapier's copy of Bevy's transform propagation systems.
///
/// See [`TransformSystem`](bevy::transform::TransformSystem::TransformPropagate).
#[derive(Debug, Hash, PartialEq, Eq, Clone, SystemSet)]
pub struct RapierTransformPropagateSet;

impl<PhysicsHooksSystemParam> Default for RapierPhysicsPlugin<PhysicsHooksSystemParam> {
    fn default() -> Self {
        Self {
            schedule: PostUpdate.intern(),
            default_system_setup: true,
            default_world_setup: Some(RapierContextInitialization { length_unit: 1f32 }),
            _phantom: PhantomData,
        }
    }
}

/// [`StageLabel`] for each phase of the plugin.
#[derive(SystemSet, Debug, Hash, PartialEq, Eq, Clone)]
pub enum PhysicsSet {
    /// This set runs the systems responsible for synchronizing (and
    /// initializing) backend data structures with current component state.
    /// These systems typically run at the after [`CoreSet::Update`].
    SyncBackend,
    /// The systems responsible for advancing the physics simulation, and
    /// updating the internal state for scene queries.
    /// These systems typically run immediately after [`PhysicsSet::SyncBackend`].
    StepSimulation,
    /// The systems responsible for updating
    /// [`crate::geometry::collider::CollidingEntities`] and writing
    /// the result of the last simulation step into our `bevy_rapier`
    /// components and the [`GlobalTransform`] component.
    /// These systems typically run immediately after [`PhysicsSet::StepSimulation`].
    Writeback,
}

impl<PhysicsHooks> Plugin for RapierPhysicsPlugin<PhysicsHooks>
where
    PhysicsHooks: 'static + BevyPhysicsHooks,
    for<'w, 's> SystemParamItem<'w, 's, PhysicsHooks>: BevyPhysicsHooks,
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
            .register_type::<SoftCcd>()
            .register_type::<GravityScale>()
            .register_type::<CollidingEntities>()
            .register_type::<Sensor>()
            .register_type::<Friction>()
            .register_type::<Restitution>()
            .register_type::<CollisionGroups>()
            .register_type::<SolverGroups>()
            .register_type::<ContactForceEventThreshold>()
            .register_type::<ContactSkin>()
            .register_type::<Group>();

        app.insert_resource(Events::<CollisionEvent>::default())
            .insert_resource(Events::<ContactForceEvent>::default())
            .insert_resource(Events::<MassModifiedEvent>::default());

        app.add_systems(
            self.schedule,
            (
                setup_rapier_configuration,
                setup_rapier_simulation_to_render_time,
            )
                .before(PhysicsSet::SyncBackend),
        );
        app.add_systems(PreStartup, insert_default_world);

        // Add each set as necessary
        if self.default_system_setup {
            app.configure_sets(
                self.schedule,
                (
                    PhysicsSet::SyncBackend,
                    PhysicsSet::StepSimulation,
                    PhysicsSet::Writeback,
                )
                    .chain()
                    .before(TransformSystem::TransformPropagate),
            );

            // These *must* be in the main schedule currently so that they do not miss events.
            app.add_systems(PostUpdate, (systems::sync_removals,));

            app.add_systems(
                self.schedule,
                (
                    Self::get_systems(PhysicsSet::SyncBackend).in_set(PhysicsSet::SyncBackend),
                    Self::get_systems(PhysicsSet::StepSimulation)
                        .in_set(PhysicsSet::StepSimulation),
                    Self::get_systems(PhysicsSet::Writeback).in_set(PhysicsSet::Writeback),
                ),
            );
            app.init_resource::<TimestepMode>();

            // Warn user if the timestep mode isn't in Fixed
            if self.schedule.as_dyn_eq().dyn_eq(FixedUpdate.as_dyn_eq()) {
                let config = app.world_mut().resource::<TimestepMode>();
                match config {
                    TimestepMode::Fixed { .. } => {}
                    mode => {
                        warn!("TimestepMode is set to `{:?}`, it is recommended to use `TimestepMode::Fixed` if you have the physics in `FixedUpdate`", mode);
                    }
                }
            }
        }
    }
}

pub struct RapierContextInitialization {
    pub length_unit: f32,
}

pub fn insert_default_world(mut commands: Commands) {
    commands.spawn((RapierContext::default(), DefaultRapierContext));
}

pub fn setup_rapier_configuration(
    mut commands: Commands,
    rapier_context: Query<(Entity, &RapierContext), Without<RapierConfiguration>>,
) {
    for (e, rapier_context) in rapier_context.iter() {
        commands.entity(e).insert(RapierConfiguration::new(
            rapier_context.integration_parameters.length_unit,
        ));
    }
}
pub fn setup_rapier_simulation_to_render_time(
    mut commands: Commands,
    rapier_context: Query<Entity, (With<RapierContext>, Without<SimulationToRenderTime>)>,
) {
    for e in rapier_context.iter() {
        commands.entity(e).insert(SimulationToRenderTime::default());
    }
}
