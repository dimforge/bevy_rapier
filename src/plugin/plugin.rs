use crate::pipeline::{CollisionEvent, ContactForceEvent};
use crate::plugin::configuration::SimulationToRenderTime;
use crate::plugin::{systems, RapierConfiguration, RapierContext};
use crate::prelude::*;
use bevy::ecs::{event::Events, schedule::SystemConfigs, system::SystemParamItem};
use bevy::prelude::*;
use std::marker::PhantomData;

pub use super::context::RapierWorld;
pub use super::context::WorldId;
pub use super::context::DEFAULT_WORLD_ID;

/// No specific user-data is associated to the hooks.
pub type NoUserData = ();

/// A plugin responsible for setting up a full Rapier physics simulation pipeline and resources.
///
/// This will automatically setup all the resources needed to run a physics simulation with the
/// Rapier physics engine.
pub struct RapierPhysicsPlugin<PhysicsHooks = ()> {
    physics_scale: f32,
    default_system_setup: bool,
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
                // Change any worlds needed before doing any calculations
                systems::apply_changing_worlds,
                // Make sure to remove any dead bodies after changing_worlds but before everything else
                // to avoid it deleting something right after adding it
                systems::sync_removals.after(systems::apply_changing_worlds),
                // Run the character controller before the manual transform propagation.
                systems::update_character_controls.after(systems::sync_removals),
                // Run Bevy transform propagation additionally to sync [`GlobalTransform`]
                bevy::transform::systems::sync_simple_transforms
                    .in_set(RapierTransformPropagateSet)
                    .after(systems::update_character_controls),
                bevy::transform::systems::propagate_transforms
                    .after(systems::update_character_controls)
                    .in_set(PropagateTransformsSet)
                    .in_set(RapierTransformPropagateSet),
            )
                .into_configs(),
            PhysicsSet::SyncBackend2 => (
                systems::init_async_colliders.after(RapierTransformPropagateSet),
                systems::apply_scale.after(systems::init_async_colliders),
                systems::apply_collider_user_changes.after(systems::apply_scale),
                systems::apply_rigid_body_user_changes.after(systems::apply_collider_user_changes),
                systems::apply_joint_user_changes.after(systems::apply_rigid_body_user_changes),
                systems::init_rigid_bodies.after(systems::apply_joint_user_changes),
                systems::sync_velocity.after(systems::init_rigid_bodies),
                systems::init_colliders
                    .after(systems::sync_velocity)
                    .after(systems::init_async_colliders),
                systems::init_joints.after(systems::init_colliders),
                systems::apply_initial_rigid_body_impulses.after(systems::init_colliders),
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

/// A set for rapier's copy of Bevy's transform propagation systems.
///
/// See [`TransformSystem`](bevy::transform::TransformSystem::TransformPropagate).
#[derive(Debug, Hash, PartialEq, Eq, Clone, SystemSet)]
pub struct RapierTransformPropagateSet;

impl<PhysicsHooksSystemParam> Default for RapierPhysicsPlugin<PhysicsHooksSystemParam> {
    fn default() -> Self {
        Self {
            physics_scale: 1.0,
            default_system_setup: true,
            _phantom: PhantomData,
        }
    }
}

/// [`StageLabel`] for each phase of the plugin.
#[derive(Debug, Hash, PartialEq, Eq, Clone, SystemSet)]
#[system_set(base)]
pub enum PhysicsSet {
    /// This set runs the systems responsible for synchronizing (and
    /// initializing) backend data structures with current component state.
    /// These systems typically run at the after [`CoreSet::Update`].
    SyncBackend,
    /// This set runs the systems responsible for synchronizing (and
    /// initializing) backend data structures with current component state.
    /// These systems typically run immediately after [`PhysicsSet::SyncBackend`].
    SyncBackend2,
    /// The copy of [`apply_system_buffers`] that runs immediately after [`PhysicsSet::SyncBackend2`].
    SyncBackendFlush,
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
            .register_type::<GravityScale>()
            .register_type::<CollidingEntities>()
            .register_type::<Sensor>()
            .register_type::<Friction>()
            .register_type::<Restitution>()
            .register_type::<CollisionGroups>()
            .register_type::<SolverGroups>()
            .register_type::<ContactForceEventThreshold>()
            .register_type::<Group>()
            .register_type::<PhysicsWorld>();

        // Insert all of our required resources. Don’t overwrite
        // the `RapierConfiguration` if it already exists.
        app.init_resource::<RapierConfiguration>();

        app.insert_resource(SimulationToRenderTime::default())
            .insert_resource(RapierContext::new(RapierWorld {
                physics_scale: self.physics_scale,
                ..Default::default()
            }))
            .insert_resource(Events::<CollisionEvent>::default())
            .insert_resource(Events::<ContactForceEvent>::default());

        // Add each set as necessary
        if self.default_system_setup {
            app.configure_sets(
                (
                    PhysicsSet::SyncBackend,
                    PhysicsSet::SyncBackend2,
                    PhysicsSet::SyncBackendFlush,
                    PhysicsSet::StepSimulation,
                    PhysicsSet::Writeback,
                )
                    .chain()
                    .after(CoreSet::UpdateFlush)
                    .before(CoreSet::PostUpdate),
            );

            app.add_systems(
                Self::get_systems(PhysicsSet::SyncBackend).in_base_set(PhysicsSet::SyncBackend),
            );

            app.add_systems(
                Self::get_systems(PhysicsSet::SyncBackend2).in_base_set(PhysicsSet::SyncBackend2),
            );

            app.add_systems(
                Self::get_systems(PhysicsSet::SyncBackendFlush)
                    .in_base_set(PhysicsSet::SyncBackendFlush),
            );
            app.add_systems(
                Self::get_systems(PhysicsSet::StepSimulation)
                    .in_base_set(PhysicsSet::StepSimulation),
            );
            app.add_systems(
                Self::get_systems(PhysicsSet::Writeback).in_base_set(PhysicsSet::Writeback),
            );
        }
    }
}
