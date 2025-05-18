use bevy::{app::PluginsState, prelude::*};
use bevy_rapier3d::prelude::*;

#[cfg(not(feature = "visual"))]
pub fn default_app() -> App {
    use bevy::time::TimeUpdateStrategy;
    let mut app = App::new();

    app.add_plugins((
        MinimalPlugins,
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
        commands.spawn((
            Camera3d::default(),
            Transform::from_xyz(-30.0, 30.0, 100.0).looking_at(Vec3::new(0.0, 0.0, 0.0), Vec3::Y),
        ));
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
