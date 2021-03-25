use bevy::prelude::*;
use rapier::pipeline::PhysicsPipeline;

pub fn setup_ui(mut commands: Commands, asset_server: Res<AssetServer>) {
    let font_handle = asset_server
        .load(format!("{}/../assets/FiraSans-Bold.ttf", env!("CARGO_MANIFEST_DIR")).as_str());
    commands
        // 2d camera
        .spawn()
        .insert_bundle(UiCameraBundle::default());
    // texture
    commands.spawn().insert_bundle(TextBundle {
        style: Style {
            align_self: AlignSelf::FlexEnd,
            ..Default::default()
        },
        text: Text {
            sections: vec![TextSection {
                value: "Physics time0.1234567890".to_string(),
                style: TextStyle {
                    font: font_handle,
                    font_size: 30.0,
                    color: Color::BLACK,
                    ..Default::default()
                },
                ..Default::default()
            }],
            ..Default::default()
        },
        ..Default::default()
    });
}

pub fn text_update_system(pipeline: Res<PhysicsPipeline>, mut query: Query<&mut Text>) {
    for mut text in query.iter_mut() {
        text.sections[0].value = format!("Physics time: {:.2}", pipeline.counters.step_time())
    }
}
