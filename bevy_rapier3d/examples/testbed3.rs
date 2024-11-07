#![allow(dead_code)]

mod boxes3;
mod debug_toggle3;
mod despawn3;
mod events3;
mod joints3;
mod joints_despawn3;
mod locked_rotations3;
mod multiple_colliders3;
mod ray_casting3;
mod static_trimesh3;

use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts, EguiPlugin};
use bevy_inspector_egui::quick::WorldInspectorPlugin;
use bevy_rapier3d::prelude::*;

#[derive(Debug, Reflect, Clone, Copy, Eq, PartialEq, Default, Hash, States)]
pub enum Examples {
    #[default]
    None,
    Boxes3,
    DebugToggle3,
    Despawn3,
    Events3,
    Joints3,
    JointsDespawn3,
    LockedRotations3,
    MultipleColliders3,
    Raycasting3,
    StaticTrimesh3,
}

#[derive(Resource, Default, Reflect)]
struct ExamplesRes {
    entities_before: Vec<Entity>,
}

#[derive(Resource, Debug, Default, Reflect)]
struct ExampleSelected(pub usize);

#[derive(Debug, Reflect)]
struct ExampleDefinition {
    pub state: Examples,
    pub name: &'static str,
}

impl From<(Examples, &'static str)> for ExampleDefinition {
    fn from((state, name): (Examples, &'static str)) -> Self {
        Self { state, name }
    }
}

#[derive(Resource, Debug, Reflect)]
struct ExampleSet(pub Vec<ExampleDefinition>);

fn main() {
    let mut app = App::new();
    app.init_resource::<ExamplesRes>()
        .add_plugins((
            DefaultPlugins,
            EguiPlugin,
            RapierPhysicsPlugin::<NoUserData>::default(),
            RapierDebugRenderPlugin::default(),
            WorldInspectorPlugin::new(),
        ))
        .register_type::<Examples>()
        .register_type::<ExamplesRes>()
        .register_type::<ExampleSelected>()
        .init_state::<Examples>()
        .insert_resource(ExampleSet(vec![
            (Examples::Boxes3, "Boxes3").into(),
            (Examples::DebugToggle3, "DebugToggle3").into(),
            (Examples::Despawn3, "Despawn3").into(),
            (Examples::Events3, "Events3").into(),
            (Examples::Joints3, "Joints3").into(),
            (Examples::JointsDespawn3, "JointsDespawn3").into(),
            (Examples::LockedRotations3, "LockedRotations3").into(),
            (Examples::MultipleColliders3, "MultipleColliders3").into(),
            (Examples::Raycasting3, "Raycasting3").into(),
            (Examples::StaticTrimesh3, "StaticTrimesh3").into(),
        ]))
        .init_resource::<ExampleSelected>()
        //
        //boxes2
        .add_systems(
            OnEnter(Examples::Boxes3),
            (boxes3::setup_graphics, boxes3::setup_physics),
        )
        .add_systems(OnExit(Examples::Boxes3), cleanup)
        //
        // Debug toggle
        .add_systems(
            OnEnter(Examples::DebugToggle3),
            (debug_toggle3::setup_graphics, debug_toggle3::setup_physics),
        )
        .add_systems(
            Update,
            (
                debug_toggle3::toggle_debug,
                (|mut debug_render_context: ResMut<DebugRenderContext>| {
                    debug_render_context.enabled = !debug_render_context.enabled;
                })
                .run_if(debug_toggle3::input_just_pressed(KeyCode::KeyV)),
            )
                .run_if(in_state(Examples::DebugToggle3)),
        )
        //
        // despawn
        .init_resource::<despawn3::DespawnResource>()
        .add_systems(
            OnEnter(Examples::Despawn3),
            (despawn3::setup_graphics, despawn3::setup_physics),
        )
        .add_systems(
            Update,
            despawn3::despawn.run_if(in_state(Examples::Despawn3)),
        )
        .add_systems(OnExit(Examples::Despawn3), cleanup)
        //
        // events
        .add_systems(
            OnEnter(Examples::Events3),
            (events3::setup_graphics, events3::setup_physics),
        )
        .add_systems(
            Update,
            events3::display_events.run_if(in_state(Examples::Events3)),
        )
        .add_systems(OnExit(Examples::Events3), cleanup)
        //
        // joints
        .add_systems(
            OnEnter(Examples::Joints3),
            (joints3::setup_graphics, joints3::setup_physics),
        )
        .add_systems(OnExit(Examples::Joints3), cleanup)
        //
        // joints despawn
        .init_resource::<joints_despawn3::DespawnResource>()
        .add_systems(
            OnEnter(Examples::JointsDespawn3),
            (
                joints_despawn3::setup_graphics,
                joints_despawn3::setup_physics,
            ),
        )
        .add_systems(
            Update,
            joints_despawn3::despawn.run_if(in_state(Examples::JointsDespawn3)),
        )
        .add_systems(OnExit(Examples::JointsDespawn3), cleanup)
        //
        // locked rotations
        .add_systems(
            OnEnter(Examples::LockedRotations3),
            (
                locked_rotations3::setup_graphics,
                locked_rotations3::setup_physics,
            ),
        )
        .add_systems(OnExit(Examples::LockedRotations3), cleanup)
        //
        // multiple colliders
        .add_systems(
            OnEnter(Examples::MultipleColliders3),
            (
                multiple_colliders3::setup_graphics,
                multiple_colliders3::setup_physics,
            ),
        )
        .add_systems(OnExit(Examples::MultipleColliders3), cleanup)
        //
        // raycasting
        .add_systems(
            OnEnter(Examples::Raycasting3),
            (ray_casting3::setup_graphics, ray_casting3::setup_physics),
        )
        .add_systems(
            Update,
            ray_casting3::cast_ray.run_if(in_state(Examples::Raycasting3)),
        )
        .add_systems(OnExit(Examples::Raycasting3), cleanup)
        //
        // static trimesh
        .init_resource::<static_trimesh3::BallState>()
        .add_systems(
            OnEnter(Examples::StaticTrimesh3),
            (
                static_trimesh3::setup_graphics,
                static_trimesh3::setup_physics,
            ),
        )
        .add_systems(
            Update,
            static_trimesh3::ball_spawner.run_if(in_state(Examples::StaticTrimesh3)),
        )
        .add_systems(OnExit(Examples::StaticTrimesh3), cleanup)
        //
        //testbed
        .add_systems(
            OnEnter(Examples::None),
            |mut next_state: ResMut<NextState<Examples>>| {
                next_state.set(Examples::Boxes3);
            },
        )
        .add_systems(OnExit(Examples::None), init)
        .add_systems(
            Update,
            (
                //ui_example_system,
                change_example.run_if(resource_changed::<ExampleSelected>),
            )
                .chain(),
        );

    app.run();
}

fn init(world: &mut World) {
    //save all entities that are in the world before setting up any example
    // to be able to always return to this state when switching from one example to the other
    world.resource_mut::<ExamplesRes>().entities_before =
        world.iter_entities().map(|e| e.id()).collect::<Vec<_>>();
}

fn cleanup(world: &mut World) {
    let keep_alive = world.resource::<ExamplesRes>().entities_before.clone();

    let remove = world
        .iter_entities()
        .filter_map(|e| (!keep_alive.contains(&e.id())).then_some(e.id()))
        .collect::<Vec<_>>();

    for r in remove {
        world.despawn(r);
    }
}

fn change_example(
    example_selected: Res<ExampleSelected>,
    examples_available: Res<ExampleSet>,
    mut next_state: ResMut<NextState<Examples>>,
) {
    next_state.set(examples_available.0[example_selected.0].state);
}
/*
fn ui_example_system(
    mut contexts: EguiContexts,
    mut current_example: ResMut<ExampleSelected>,
    examples_available: Res<ExampleSet>,
) {
    egui::Window::new("Testbed").show(contexts.ctx_mut(), |ui| {
        let mut changed = false;
        egui::ComboBox::from_label("example")
            .width(150.0)
            .selected_text(examples_available.0[current_example.0].name)
            .show_ui(ui, |ui| {
                for (id, value) in examples_available.0.iter().enumerate() {
                    changed = ui
                        .selectable_value(&mut current_example.0, id, value.name)
                        .changed()
                        || changed;
                }
            });
        if ui.button("Next").clicked() {
            current_example.0 = (current_example.0 + 1) % examples_available.0.len();
        }
    });
}
  */
