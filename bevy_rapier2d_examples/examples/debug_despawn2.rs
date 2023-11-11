// NOTE: this demo is great for debugging despawning.
//       It was extracted for one of the debug branch from @audunhalland
//       in https://github.com/dimforge/bevy_rapier/issues/75

use bevy::prelude::*;
use bevy_rapier2d::prelude::*;
use bevy_rapier2d_examples::{ExamplePluginDebugDespawn2, ExampleResource, Examples};

fn main() {
    App::new()
        .add_state::<Examples>()
        .insert_resource(ExampleResource::default())
        .add_plugins((
            DefaultPlugins,
            RapierPhysicsPlugin::<NoUserData>::pixels_per_meter(100.0),
            RapierDebugRenderPlugin::default(),
            ExamplePluginDebugDespawn2::default(),
        ))
        .add_systems(Startup, set_state)
        .run();
}

fn set_state(mut app_state: ResMut<NextState<Examples>>) {
    app_state.set(Examples::DebugDespawn2);
}
