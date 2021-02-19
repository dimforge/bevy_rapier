use bevy::prelude::*;
use rapier::pipeline::PhysicsPipeline;

pub fn setup_ui(commands: &mut Commands, asset_server: Res<AssetServer>) {
    let font_handle = asset_server
        .load(format!("{}/../assets/FiraSans-Bold.ttf", env!("CARGO_MANIFEST_DIR")).as_str());
    commands
        // 2d camera
        .spawn(UiCameraBundle::default())
        // texture
        .spawn(TextBundle {
            style: Style {
                align_self: AlignSelf::FlexEnd,
                ..Default::default()
            },
            text: Text::with_section(
                "Physics time0.1234567890".to_string(),
                TextStyle {
                    font_size: 30.0,
                    color: Color::BLACK,
                    font: font_handle,
                },
                Default::default(),
            ),
            ..Default::default()
        });
}

pub fn text_update_system(pipeline: Res<PhysicsPipeline>, mut query: Query<&mut Text>) {
    for mut text in query.iter_mut() {
        text.sections[0].value = format!("Physics time: {:.2}", pipeline.counters.step_time());
    }
}
