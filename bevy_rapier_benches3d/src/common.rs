use bevy::{
    app::PluginsState,
    log::LogPlugin,
    prelude::*,
    render::{
        settings::{RenderCreation, WgpuSettings},
        RenderPlugin,
    },
    scene::ScenePlugin,
    time::TimeUpdateStrategy,
};
use bevy_rapier3d::prelude::*;

#[cfg(not(feature = "visual"))]
pub fn default_app() -> App {
    let mut app = App::new();

    app.add_plugins((
        WindowPlugin::default(),
        MinimalPlugins,
        AssetPlugin::default(),
        ScenePlugin,
        LogPlugin::default(),
        RenderPlugin {
            render_creation: RenderCreation::Automatic(WgpuSettings {
                backends: None,
                ..Default::default()
            }),
            ..Default::default()
        },
        ImagePlugin::default(),
        HierarchyPlugin,
        TransformPlugin,
        RapierPhysicsPlugin::<()>::default(),
    ));

    // 60 physics
    app.insert_resource(TimeUpdateStrategy::ManualDuration(
        std::time::Duration::from_secs_f32(1f32 / 60f32),
    ));
    app
}

#[cfg(feature = "visual")]
pub fn default_app() -> App {
    use bevy::diagnostic::{FrameTimeDiagnosticsPlugin, LogDiagnosticsPlugin};

    let mut app = App::new();
    println!("visual mode!");
    app.insert_resource(ClearColor(Color::srgb(
        0xF9 as f32 / 255.0,
        0xF9 as f32 / 255.0,
        0xFF as f32 / 255.0,
    )));
    app.add_plugins((
        DefaultPlugins,
        RapierPhysicsPlugin::<()>::default(),
        RapierDebugRenderPlugin::default(),
        FrameTimeDiagnosticsPlugin::default(),
        LogDiagnosticsPlugin::default(),
    ));
    app.add_systems(Startup, |mut commands: Commands| {
        commands.spawn(Camera3dBundle {
            transform: Transform::from_xyz(-30.0, 30.0, 100.0)
                .looking_at(Vec3::new(0.0, 0.0, 0.0), Vec3::Y),
            ..Default::default()
        });
    });
    app
}

pub fn wait_app_start(app: &mut App) {
    while app.plugins_state() != PluginsState::Ready {
        bevy::tasks::tick_global_task_pools_on_main_thread();
    }

    app.finish();
    app.cleanup();
}
