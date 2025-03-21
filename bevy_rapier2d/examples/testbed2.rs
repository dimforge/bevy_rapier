// FIXME: Uncomment once `bevy_egui` and `bevy_inspector_egui` get updated to be compatible with bevy 0.16
// #![allow(dead_code)]

// mod boxes2;
// mod debug_despawn2;
// mod debug_toggle2;
// mod despawn2;
// mod events2;
// mod joints2;
// mod joints_despawn2;
// mod locked_rotations2;
// mod multiple_colliders2;
// mod player_movement2;
// mod rope_joint2;

// use bevy::prelude::*;
// use bevy_egui::{egui, EguiContexts, EguiPlugin};
// use bevy_inspector_egui::quick::WorldInspectorPlugin;
// use bevy_rapier2d::prelude::*;

// #[derive(Debug, Reflect, Clone, Copy, Eq, PartialEq, Default, Hash, States)]
// pub enum Examples {
//     #[default]
//     None,
//     Boxes2,
//     DebugToggle2,
//     RopeJoint2,
//     DebugDespawn2,
//     Despawn2,
//     Events2,
//     Joints2,
//     JointsDespawn2,
//     LockedRotations2,
//     MultipleColliders2,
//     PlayerMovement2,
// }

// #[derive(Resource, Default, Reflect)]
// struct ExamplesRes {
//     entities_before: Vec<Entity>,
// }

// #[derive(Resource, Debug, Default, Reflect)]
// struct ExampleSelected(pub usize);

// #[derive(Debug, Reflect)]
// struct ExampleDefinition {
//     pub state: Examples,
//     pub name: &'static str,
// }

// impl From<(Examples, &'static str)> for ExampleDefinition {
//     fn from((state, name): (Examples, &'static str)) -> Self {
//         Self { state, name }
//     }
// }

// #[derive(Resource, Debug, Reflect)]
// struct ExampleSet(pub Vec<ExampleDefinition>);

// fn main() {
//     let mut app = App::new();
//     app.init_resource::<ExamplesRes>()
//         .add_plugins((
//             DefaultPlugins,
//             EguiPlugin,
//             RapierPhysicsPlugin::<NoUserData>::pixels_per_meter(10.0),
//             RapierDebugRenderPlugin::default(),
//             WorldInspectorPlugin::new(),
//         ))
//         .register_type::<Examples>()
//         .register_type::<ExamplesRes>()
//         .register_type::<ExampleSelected>()
//         .init_state::<Examples>()
//         .insert_resource(ExampleSet(vec![
//             (Examples::Boxes2, "Boxes3").into(),
//             (Examples::RopeJoint2, "RopeJoint2").into(),
//             (Examples::DebugDespawn2, "DebugDespawn2").into(),
//             (Examples::Despawn2, "Despawn3").into(),
//             (Examples::Events2, "Events3").into(),
//             (Examples::Joints2, "Joints3").into(),
//             (Examples::JointsDespawn2, "JointsDespawn3").into(),
//             (Examples::LockedRotations2, "LockedRotations3").into(),
//             (Examples::MultipleColliders2, "MultipleColliders3").into(),
//             (Examples::PlayerMovement2, "PlayerMovement2").into(),
//         ]))
//         .init_resource::<ExampleSelected>()
//         //
//         //boxes2
//         .add_systems(
//             OnEnter(Examples::Boxes2),
//             (boxes2::setup_graphics, boxes2::setup_physics),
//         )
//         .add_systems(OnExit(Examples::Boxes2), cleanup)
//         //
//         // Debug toggle
//         .add_systems(
//             OnEnter(Examples::DebugToggle2),
//             (debug_toggle2::setup_graphics, debug_toggle2::setup_physics),
//         )
//         .add_systems(
//             Update,
//             (
//                 debug_toggle2::toggle_debug,
//                 (|mut debug_render_context: ResMut<DebugRenderContext>| {
//                     debug_render_context.enabled = !debug_render_context.enabled;
//                 })
//                 .run_if(debug_toggle2::input_just_pressed(KeyCode::KeyV)),
//             )
//                 .run_if(in_state(Examples::DebugToggle2)),
//         )
//         .add_systems(OnExit(Examples::DebugToggle2), cleanup)
//         //
//         // rope joint
//         .add_systems(
//             OnEnter(Examples::RopeJoint2),
//             (rope_joint2::setup_graphics, rope_joint2::setup_physics),
//         )
//         .add_systems(OnExit(Examples::RopeJoint2), cleanup)
//         //
//         //debug despawn
//         .init_resource::<debug_despawn2::Game>()
//         .add_systems(OnEnter(Examples::DebugDespawn2), debug_despawn2::setup_game)
//         .add_systems(
//             Update,
//             debug_despawn2::cube_sleep_detection.run_if(in_state(Examples::DebugDespawn2)),
//         )
//         .add_systems(OnExit(Examples::DebugDespawn2), cleanup)
//         //
//         //despawn2
//         .insert_resource(despawn2::DespawnResource::default())
//         .insert_resource(despawn2::ResizeResource::default())
//         .add_systems(
//             OnEnter(Examples::Despawn2),
//             (despawn2::setup_graphics, despawn2::setup_physics),
//         )
//         .add_systems(
//             Update,
//             (despawn2::despawn, despawn2::resize).run_if(in_state(Examples::Despawn2)),
//         )
//         .add_systems(OnExit(Examples::Despawn2), cleanup)
//         //
//         //events
//         .add_systems(
//             OnEnter(Examples::Events2),
//             (events2::setup_graphics, events2::setup_physics),
//         )
//         .add_systems(
//             Update,
//             events2::display_events.run_if(in_state(Examples::Events2)),
//         )
//         .add_systems(OnExit(Examples::Events2), cleanup)
//         //
//         //events
//         .add_systems(
//             OnEnter(Examples::Joints2),
//             (joints2::setup_graphics, joints2::setup_physics),
//         )
//         .add_systems(OnExit(Examples::Joints2), cleanup)
//         //
//         //despawn2
//         .insert_resource(joints_despawn2::DespawnResource::default())
//         .add_systems(
//             OnEnter(Examples::JointsDespawn2),
//             (
//                 joints_despawn2::setup_graphics,
//                 joints_despawn2::setup_physics,
//             ),
//         )
//         .add_systems(
//             Update,
//             (joints_despawn2::despawn).run_if(in_state(Examples::JointsDespawn2)),
//         )
//         .add_systems(OnExit(Examples::JointsDespawn2), cleanup)
//         //
//         //locked rotations
//         .add_systems(
//             OnEnter(Examples::LockedRotations2),
//             (
//                 locked_rotations2::setup_graphics,
//                 locked_rotations2::setup_physics,
//             ),
//         )
//         .add_systems(OnExit(Examples::LockedRotations2), cleanup)
//         //
//         //multiple colliders
//         .add_systems(
//             OnEnter(Examples::MultipleColliders2),
//             (
//                 multiple_colliders2::setup_graphics,
//                 multiple_colliders2::setup_physics,
//             ),
//         )
//         .add_systems(OnExit(Examples::MultipleColliders2), cleanup)
//         //
//         //player movement
//         .add_systems(
//             OnEnter(Examples::PlayerMovement2),
//             player_movement2::spawn_player,
//         )
//         .add_systems(
//             Update,
//             (player_movement2::player_movement).run_if(in_state(Examples::PlayerMovement2)),
//         )
//         .add_systems(
//             OnExit(Examples::PlayerMovement2),
//             (
//                 cleanup,
//                 |mut rapier_config: Query<&mut RapierConfiguration>,
//                  ctxt: ReadRapierContext|
//                  -> Result<()> {
//                     let mut rapier_config = rapier_config.single_mut()?;
//                     rapier_config.gravity = RapierConfiguration::new(
//                         ctxt.single()?.simulation.integration_parameters.length_unit,
//                     )
//                     .gravity;
//                     Ok(())
//                 },
//             ),
//         )
//         //
//         //testbed
//         .add_systems(
//             OnEnter(Examples::None),
//             |mut next_state: ResMut<NextState<Examples>>| {
//                 next_state.set(Examples::Boxes2);
//             },
//         )
//         .add_systems(OnExit(Examples::None), init)
//         .add_systems(
//             Update,
//             (
//                 ui_example_system,
//                 change_example.run_if(resource_changed::<ExampleSelected>),
//             )
//                 .chain(),
//         );

//     app.run();
// }

// fn init(world: &mut World) {
//     //save all entities that are in the world before setting up any example
//     // to be able to always return to this state when switching from one example to the other
//     world.resource_mut::<ExamplesRes>().entities_before =
//         world.iter_entities().map(|e| e.id()).collect::<Vec<_>>();
// }

// fn cleanup(world: &mut World) {
//     let keep_alive = world.resource::<ExamplesRes>().entities_before.clone();

//     let remove = world
//         .iter_entities()
//         .filter_map(|e| (!keep_alive.contains(&e.id())).then_some(e.id()))
//         .collect::<Vec<_>>();

//     for r in remove {
//         world.despawn(r);
//     }
// }

// fn change_example(
//     example_selected: Res<ExampleSelected>,
//     examples_available: Res<ExampleSet>,
//     mut next_state: ResMut<NextState<Examples>>,
// ) {
//     next_state.set(examples_available.0[example_selected.0].state);
// }

// fn ui_example_system(
//     mut contexts: EguiContexts,
//     mut current_example: ResMut<ExampleSelected>,
//     examples_available: Res<ExampleSet>,
// ) {
//     egui::Window::new("Testbed").show(contexts.ctx_mut(), |ui| {
//         let mut changed = false;
//         egui::ComboBox::from_label("example")
//             .width(150.0)
//             .selected_text(examples_available.0[current_example.0].name)
//             .show_ui(ui, |ui| {
//                 for (id, value) in examples_available.0.iter().enumerate() {
//                     changed = ui
//                         .selectable_value(&mut current_example.0, id, value.name)
//                         .changed()
//                         || changed;
//                 }
//             });
//         if ui.button("Next").clicked() {
//             current_example.0 = (current_example.0 + 1) % examples_available.0.len();
//         }
//     });
// }
