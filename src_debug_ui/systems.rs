use bevy::prelude::*;
use rapier::pipeline::PhysicsPipeline;

pub fn setup_ui(mut commands: Commands, asset_server: Res<AssetServer>) {
    let font_handle = asset_server
        .load(format!("{}/../assets/FiraSans-Bold.ttf", env!("CARGO_MANIFEST_DIR")).as_str());
    commands
        // 2d camera
        .spawn(UiCameraComponents::default())
        // texture
        .spawn(TextComponents {
            style: Style {
                align_self: AlignSelf::FlexEnd,
                ..Default::default()
            },
            text: Text {
                value: "Physics time0.1234567890".to_string(),
                font: font_handle,
                style: TextStyle {
                    font_size: 30.0,
                    color: Color::BLACK,
                },
            },
            ..Default::default()
        });
}

pub fn text_update_system(pipeline: Res<PhysicsPipeline>, mut query: Query<&mut Text>) {
    for mut text in query.iter_mut() {
        text.value = format!("Physics time: {:.2}", pipeline.counters.step_time())
    }
}
