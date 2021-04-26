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
    commands.spawn_bundle(TextBundle {
        style: Style {
            align_self: AlignSelf::FlexEnd,
            ..Default::default()
        },
        text: Text {
            sections: vec![TextSection {
                value: "Physics time0.1234567890".to_string(),
                style: TextStyle {
                    font: font_handle,
                    font_size: 15.0,
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
    let profile_string = format!(
        r#"Total: {:.2}ms
Collision detection: {:.2}ms
|_ Broad-phase: {:.2}ms
   Narrow-phase: {:.2}ms
Island computation: {:.2}ms
Solver: {:.2}ms
|_ Velocity assembly: {:.2}ms
   Velocity resolution: {:.2}ms
   Velocity integration: {:.2}ms
   Position assembly: {:.2}ms
   Position resolution: {:.2}ms
CCD: {:.2}ms
|_ # of substeps: {}
   TOI computation: {:.2}ms
   Broad-phase: {:.2}ms
   Narrow-phase: {:.2}ms
   Solver: {:.2}ms"#,
        pipeline.counters.step_time(),
        pipeline.counters.collision_detection_time(),
        pipeline.counters.broad_phase_time(),
        pipeline.counters.narrow_phase_time(),
        pipeline.counters.island_construction_time(),
        pipeline.counters.solver_time(),
        pipeline.counters.solver.velocity_assembly_time.time(),
        pipeline.counters.velocity_resolution_time(),
        pipeline.counters.solver.velocity_update_time.time(),
        pipeline.counters.solver.position_assembly_time.time(),
        pipeline.counters.position_resolution_time(),
        pipeline.counters.ccd_time(),
        pipeline.counters.ccd.num_substeps,
        pipeline.counters.ccd.toi_computation_time.time(),
        pipeline.counters.ccd.broad_phase_time.time(),
        pipeline.counters.ccd.narrow_phase_time.time(),
        pipeline.counters.ccd.solver_time.time(),
    );

    for mut text in query.iter_mut() {
        text.sections[0].value = profile_string.clone();
    }
}
