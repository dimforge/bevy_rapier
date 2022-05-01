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
    _phantom: PhantomData<PhysicsHooksData>,
}

impl<PhysicsHooksData> RapierPhysicsPlugin<PhysicsHooksData> {
    /// Specifies a scale ratio between the physics world and the bevy transforms.
    ///
    /// This affects the size of every elements in the physics engine, by multiplying
    /// all the length-related quantities by the `physics_scale` factor. This should
    /// likely always be 1.0 in 3D. In 2D, this is useful to specify a "pixels-per-meter"
    /// conversion ratio.
    pub fn with_physics_scale(physics_scale: f32) -> Self {
        Self {
            physics_scale,
            _phantom: PhantomData,
        }
    }

    /// Specifies how many pixels on the 2D canvas equal one meter on the physics world.
    ///
    /// This conversion unit assumes that the 2D camera uses an unscaled projection.
    #[cfg(feature = "dim2")]
    pub fn pixels_per_meter(pixels_per_meter: f32) -> Self {
        Self {
            physics_scale: pixels_per_meter,
            _phantom: PhantomData,
        }
    }
}

#[cfg(feature = "dim3")]
impl<PhysicsHooksData> Default for RapierPhysicsPlugin<PhysicsHooksData> {
    fn default() -> Self {
        Self {
            physics_scale: 1.0,
            _phantom: PhantomData,
        }
    }
}

#[derive(Debug, Hash, PartialEq, Eq, Clone, StageLabel)]
pub enum PhysicsStages {
    StepSimulation,
    DetectDespawn,
}

impl<PhysicsHooksData: 'static + WorldQuery + Send + Sync> Plugin
    for RapierPhysicsPlugin<PhysicsHooksData>
{
    fn build(&self, app: &mut App) {
        let mut context = RapierContext::default();
        context.physics_scale = self.physics_scale;

        app.add_stage_before(
            CoreStage::Update,
            PhysicsStages::StepSimulation,
            SystemStage::parallel(),
        );
        app.add_stage_before(
            CoreStage::Last,
            PhysicsStages::DetectDespawn,
            SystemStage::parallel(),
        );
        app.insert_resource(RapierConfiguration::default())
            .insert_resource(SimulationToRenderTime::default())
            .insert_resource(context)
            .insert_resource(Events::<CollisionEvent>::default())
            .add_system_set_to_stage(
                PhysicsStages::StepSimulation,
                SystemSet::new()
                    .with_system(systems::init_async_shapes)
                    .with_system(systems::apply_scale.after(systems::init_async_shapes))
                    .with_system(systems::apply_collider_user_changes.after(systems::apply_scale))
                    .with_system(
                        systems::apply_rigid_body_user_changes
                            .after(systems::apply_collider_user_changes),
                    )
                    .with_system(
                        systems::apply_joint_user_changes
                            .after(systems::apply_rigid_body_user_changes),
                    )
                    .with_system(
                        systems::init_rigid_bodies.after(systems::apply_joint_user_changes),
                    )
                    .with_system(
                        systems::init_colliders
                            .after(systems::init_rigid_bodies)
                            .after(systems::init_async_shapes),
                    )
                    .with_system(systems::init_joints.after(systems::init_colliders))
                    .with_system(systems::sync_removals.after(systems::init_joints))
                    .with_system(
                        systems::step_simulation::<PhysicsHooksData>.after(systems::sync_removals),
                    )
                    .with_system(
                        systems::writeback_rigid_bodies
                            .after(systems::step_simulation::<PhysicsHooksData>),
                    ),
            )
            // NOTE: we run sync_removals at the end of the frame to, in order to make sure we don’t miss any `RemovedComponents`.
            .add_system_to_stage(PhysicsStages::DetectDespawn, systems::sync_removals);

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
