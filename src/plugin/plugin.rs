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
use rapier::dynamics::IntegrationParameters;
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
    /// Read during [`RapierPhysicsPlugin::build()`],
    /// to help initializing [`RapierContextInitialization`] resource.
    /// This will be ignored if that resource already exists.
    default_world_setup: RapierContextInitialization,
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
        self.default_world_setup =
            RapierContextInitialization::InitializeDefaultRapierContext { length_unit };
        self
    }

    /// Specifies a default world initialization strategy.
    ///
    /// The default is to initialize a [`RapierContext`] with a length unit of 1.
    pub fn with_custom_initialization(
        mut self,
        default_world_initialization: RapierContextInitialization,
    ) -> Self {
        self.default_world_setup = default_world_initialization;
        self
    }

    /// Specifies whether the plugin should setup each of its [`PhysicsSet`]
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
            default_world_setup: RapierContextInitialization::InitializeDefaultRapierContext {
                length_unit: pixels_per_meter,
            },
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
    /// [`with_default_system_setup(false)`](Self::with_default_system_setup).
    /// See [`PhysicsSet`] for a description of these systems.
    pub fn get_systems(set: PhysicsSet) -> SystemConfigs {
        match set {
            PhysicsSet::SyncBackend => (
                // Run the character controller before the manual transform propagation.
                systems::update_character_controls.in_set(PhysicsSet::SyncBackend),
                // Run Bevy transform propagation additionally to sync [`GlobalTransform`]
                (
                    bevy::transform::systems::sync_simple_transforms,
                    bevy::transform::systems::propagate_transforms,
                )
                    .chain()
                    .in_set(RapierTransformPropagateSet),
                (
                    systems::on_add_entity_with_parent,
                    systems::on_change_world,
                    systems::sync_removals,
                    #[cfg(all(feature = "dim3", feature = "async-collider"))]
                    systems::init_async_scene_colliders,
                    #[cfg(all(feature = "dim3", feature = "async-collider"))]
                    systems::init_async_colliders,
                    systems::init_rigid_bodies,
                    systems::init_colliders,
                    systems::init_joints,
                    systems::apply_scale,
                    systems::apply_collider_user_changes,
                    systems::apply_rigid_body_user_changes,
                    systems::apply_joint_user_changes,
                    systems::apply_initial_rigid_body_impulses,
                )
                    .chain()
                    .in_set(PhysicsSet::SyncBackend),
            )
                .chain()
                .into_configs(),
            PhysicsSet::StepSimulation => (systems::step_simulation::<PhysicsHooks>)
                .in_set(PhysicsSet::StepSimulation)
                .into_configs(),
            PhysicsSet::Writeback => (
                systems::update_colliding_entities,
                systems::writeback_rigid_bodies,
                // Each writeback write to different properties.
                systems::writeback_mass_properties.ambiguous_with(systems::writeback_rigid_bodies),
            )
                .in_set(PhysicsSet::Writeback)
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
            default_world_setup: default(),
            _phantom: PhantomData,
        }
    }
}

/// [`SystemSet`] for each phase of the plugin.
#[derive(SystemSet, Debug, Hash, PartialEq, Eq, Clone)]
pub enum PhysicsSet {
    /// This set runs the systems responsible for synchronizing (and
    /// initializing) backend data structures with current component state.
    /// These systems typically run at the after [`Update`].
    SyncBackend,
    /// The systems responsible for advancing the physics simulation, and
    /// updating the internal state for scene queries.
    /// These systems typically run immediately after [`PhysicsSet::SyncBackend`].
    StepSimulation,
    /// The systems responsible for updating
    /// [`crate::geometry::CollidingEntities`] and writing
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
            .register_type::<Group>()
            .register_type::<RapierContextEntityLink>()
            .register_type::<ColliderDebugColor>()
            .register_type::<RapierConfiguration>()
            .register_type::<SimulationToRenderTime>()
            .register_type::<DefaultRapierContext>()
            .register_type::<RapierContextInitialization>()
            .register_type::<ColliderDebugColor>();

        app.insert_resource(Events::<CollisionEvent>::default())
            .insert_resource(Events::<ContactForceEvent>::default())
            .insert_resource(Events::<MassModifiedEvent>::default());
        let default_world_init = app.world().get_resource::<RapierContextInitialization>();
        if let Some(world_init) = default_world_init {
            warn!("RapierPhysicsPlugin added but a `RapierContextInitialization` resource was already existing.\
            This might overwrite previous configuration made via `RapierPhysicsPlugin::with_custom_initialization`\
            or `RapierPhysicsPlugin::with_length_unit`.
            The following resource will be used: {:?}", world_init);
        } else {
            app.insert_resource(self.default_world_setup.clone());
        }

        // TODO: feature gate this
        if !app.is_plugin_added::<picking_backend::RapierPickingPlugin>() {
            app.add_plugins(picking_backend::RapierPickingPlugin::default());
        }

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
            app.configure_sets(
                self.schedule,
                RapierTransformPropagateSet.in_set(PhysicsSet::SyncBackend),
            );

            app.add_systems(
                self.schedule,
                (
                    Self::get_systems(PhysicsSet::SyncBackend),
                    Self::get_systems(PhysicsSet::StepSimulation),
                    Self::get_systems(PhysicsSet::Writeback),
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

/// Specifies a default configuration for the default [`RapierContext`]
///
/// Designed to be passed as parameter to [`RapierPhysicsPlugin::with_custom_initialization`].
#[derive(Resource, Debug, Reflect, Clone)]
pub enum RapierContextInitialization {
    /// [`RapierPhysicsPlugin`] will not spawn any entity containing [`RapierContext`] automatically.
    ///
    /// You are responsible for creating a [`RapierContext`],
    /// before spawning any rapier entities (rigidbodies, colliders, joints).
    ///
    /// You might be interested in adding [`DefaultRapierContext`] to the created world.
    NoAutomaticRapierContext,
    /// [`RapierPhysicsPlugin`] will spawn an entity containing a [`RapierContext`]
    /// automatically during [`PreStartup`], with the [`DefaultRapierContext`] marker component.
    InitializeDefaultRapierContext {
        /// See [`IntegrationParameters::length_unit`]
        length_unit: f32,
    },
}

impl Default for RapierContextInitialization {
    fn default() -> Self {
        RapierContextInitialization::InitializeDefaultRapierContext { length_unit: 1f32 }
    }
}

pub fn insert_default_world(
    mut commands: Commands,
    initialization_data: Res<RapierContextInitialization>,
) {
    match initialization_data.as_ref() {
        RapierContextInitialization::NoAutomaticRapierContext => {}
        RapierContextInitialization::InitializeDefaultRapierContext { length_unit } => {
            commands.spawn((
                Name::new("Rapier Context"),
                RapierContext {
                    integration_parameters: IntegrationParameters {
                        length_unit: *length_unit,
                        ..default()
                    },
                    ..RapierContext::default()
                },
                DefaultRapierContext,
            ));
        }
    }
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

#[cfg(test)]
mod test {

    use bevy::{
        ecs::schedule::Stepping,
        prelude::Component,
        time::{TimePlugin, TimeUpdateStrategy},
    };
    use rapier::{data::Index, dynamics::RigidBodyHandle};
    use systems::tests::HeadlessRenderPlugin;

    use crate::{plugin::*, prelude::*};

    #[cfg(feature = "dim3")]
    fn cuboid(hx: Real, hy: Real, hz: Real) -> Collider {
        Collider::cuboid(hx, hy, hz)
    }
    #[cfg(feature = "dim2")]
    fn cuboid(hx: Real, hy: Real, _hz: Real) -> Collider {
        Collider::cuboid(hx, hy)
    }

    #[derive(Component)]
    pub struct TestMarker;

    #[test]
    pub fn hierarchy_link_propagation() {
        return main();

        use bevy::prelude::*;

        fn run_test(app: &mut App) {
            app.insert_resource(TimeUpdateStrategy::ManualDuration(
                std::time::Duration::from_secs_f32(1f32 / 60f32),
            ));

            app.add_systems(Update, setup_physics);

            let mut stepping = Stepping::new();

            app.update();

            stepping
                .add_schedule(PostUpdate)
                .add_schedule(Update)
                .enable()
                .set_breakpoint(PostUpdate, systems::on_add_entity_with_parent)
                .set_breakpoint(PostUpdate, systems::init_rigid_bodies)
                .set_breakpoint(PostUpdate, systems::on_change_world)
                .set_breakpoint(PostUpdate, systems::sync_removals)
                .set_breakpoint(Update, setup_physics);

            app.insert_resource(stepping);

            let mut stepping = app.world_mut().resource_mut::<Stepping>();
            // Advancing once to get the context.
            stepping.continue_frame();
            app.update();
            // arbitrary hardcoded amount to run the simulation for a few frames.
            // This test uses stepping so the actual amount of frames is this `number / breakpoints`
            for _ in 0..20 {
                let world = app.world_mut();
                let stepping = world.resource_mut::<Stepping>();
                if let Some(cursor) = &stepping.cursor() {
                    let system = world
                        .resource::<Schedules>()
                        .get(cursor.0)
                        .unwrap()
                        .systems()
                        .unwrap()
                        .find(|s| s.0 == cursor.1)
                        .unwrap();
                    println!(
                        "next system: {}",
                        system
                            .1
                            .name()
                            .to_string()
                            .split_terminator("::")
                            .last()
                            .unwrap()
                    );
                } else {
                    println!("no cursor, new frame!");
                }
                let mut stepping = world.resource_mut::<Stepping>();
                stepping.continue_frame();
                app.update();
                let context = app
                    .world_mut()
                    .query::<&RapierContext>()
                    .get_single(&app.world())
                    .unwrap();

                println!("{:?}", &context.entity2body);
            }
            let context = app
                .world_mut()
                .query::<&RapierContext>()
                .get_single(&app.world())
                .unwrap();

            assert_eq!(
                context.entity2body.iter().next().unwrap().1,
                // assert the generation is 0, that means we didn't modify it twice (due to change world detection)
                &RigidBodyHandle(Index::from_raw_parts(0, 0))
            );
        }

        fn main() {
            let mut app = App::new();
            app.add_plugins((
                HeadlessRenderPlugin,
                TransformPlugin,
                TimePlugin,
                RapierPhysicsPlugin::<NoUserData>::default(),
            ));
            run_test(&mut app);
        }

        pub fn setup_physics(mut commands: Commands, mut counter: Local<i32>) {
            // run on the 3rd iteration: I believe current logic is:
            // - 1st is to setup bevy internals
            // - 2nd is to wait for having stepping enabled, as I couldnt get it to work for the first update.
            // - 3rd, we're looking to test adding a rapier entity while playing, opposed to a Startup function,
            //   which most examples are focused on.
            let run_at = 3;
            if *counter == run_at {
                return;
            }
            *counter += 1;
            if *counter < run_at {
                return;
            }

            commands.spawn((
                Transform::from_xyz(0.0, 13.0, 0.0),
                RigidBody::Dynamic,
                cuboid(0.5, 0.5, 0.5),
                TestMarker,
            ));
        }
    }
}
