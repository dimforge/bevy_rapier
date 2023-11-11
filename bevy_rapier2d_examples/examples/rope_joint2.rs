use bevy::prelude::*;
use bevy_rapier2d::prelude::*;
use bevy_rapier2d_examples::{ExamplePluginRopeJoint2, ExampleResource, Examples};

fn main() {
    App::new()
        .add_state::<Examples>()
        .insert_resource(ExampleResource::default())
        .add_plugins((
            DefaultPlugins,
            RapierPhysicsPlugin::<NoUserData>::pixels_per_meter(100.0),
            RapierDebugRenderPlugin::default(),
            ExamplePluginRopeJoint2::default(),
        ))
        .add_systems(Startup, set_state)
        .run();
}

fn set_state(mut app_state: ResMut<NextState<Examples>>) {
    app_state.set(Examples::RopeJoint2);
}
