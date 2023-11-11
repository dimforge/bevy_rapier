use bevy::prelude::*;
use bevy_rapier2d::prelude::*;
use bevy_rapier2d_examples::{
    ExamplePluginBoxes2, ExamplePluginDebugDespawn2, ExamplePluginDespawn2,
    ExamplePluginRopeJoint2, ExampleResource, Examples,
};

fn main() {
    App::new()
        .add_state::<Examples>()
        .insert_resource(ExampleResource::default())
        .add_plugins((
            DefaultPlugins,
            RapierPhysicsPlugin::<NoUserData>::pixels_per_meter(100.0),
            RapierDebugRenderPlugin::default(),
            ExamplePluginBoxes2::default(),
            ExamplePluginRopeJoint2::default(),
            ExamplePluginDebugDespawn2::default(),
            ExamplePluginDespawn2::default(),
        ))
        .add_systems(Update, check_toggle)
        .run();
}

fn check_toggle(
    state: Res<State<Examples>>,
    mut next_state: ResMut<NextState<Examples>>,
    mouse_input: Res<Input<MouseButton>>,
) {
    if mouse_input.just_pressed(MouseButton::Left) {
        let next = match *state.get() {
            Examples::Boxes2 => Examples::RopeJoint2,
            Examples::RopeJoint2 => Examples::DebugDespawn2,
            Examples::DebugDespawn2 => Examples::Despawn2,
            Examples::Despawn2 => Examples::Boxes2,
        };
        next_state.set(next);
    }
}
