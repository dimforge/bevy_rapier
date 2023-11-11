mod boxes2;
mod debug_despawn2;
mod despawn2;
mod rope_joint2;

pub use boxes2::ExamplePluginBoxes2;
pub use debug_despawn2::ExamplePluginDebugDespawn2;
pub use despawn2::ExamplePluginDespawn2;
pub use rope_joint2::ExamplePluginRopeJoint2;

use bevy::prelude::*;

#[derive(Debug, Clone, Copy, Default, Eq, PartialEq, Hash, States)]
pub enum Examples {
    #[default]
    Boxes2,
    RopeJoint2,
    DebugDespawn2,
    Despawn2,
}

#[derive(Default, Resource)]
pub struct ExampleResource {
    pub root: Option<Entity>,
    pub camera: Option<Entity>,
}

pub fn cleanup_resource(mut commands: Commands, res: Res<ExampleResource>) {
    if let Some(e) = res.root {
        commands.entity(e).despawn_recursive();
    }
    if let Some(e) = res.camera {
        commands.entity(e).despawn_recursive();
    }
}
