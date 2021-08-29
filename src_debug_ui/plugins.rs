use crate::ui::systems;
use bevy::prelude::*;

pub struct DebugUiPlugin;

impl Plugin for DebugUiPlugin {
    fn build(&self, app: &mut App) {
        app.add_startup_system(systems::setup_ui.system())
            .add_system_to_stage(CoreStage::Update, systems::text_update_system.system());
    }
}
