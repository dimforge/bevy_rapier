use bevy::{
    app::PluginsState,
    prelude::*,
    render::{
        settings::{RenderCreation, WgpuSettings},
        RenderPlugin,
    },
    scene::ScenePlugin,
    time::TimeUpdateStrategy,
};
use bevy_rapier3d::prelude::*;

pub fn default_app() -> App {
    let mut app = App::new();

    app.add_plugins((
        WindowPlugin::default(),
        MinimalPlugins,
        AssetPlugin::default(),
        ScenePlugin,
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

pub fn wait_app_start(app: &mut App) {
    while app.plugins_state() != PluginsState::Ready {
        bevy::tasks::tick_global_task_pools_on_main_thread();
    }

    app.finish();
    app.cleanup();
}

pub fn bench_app(bencher: divan::Bencher, steps: u32, setup: impl Fn(&mut App)) {
    bencher
        .with_inputs(|| {
            let mut app = default_app();
            setup(&mut app);
            wait_app_start(&mut app);
            app
        })
        .bench_local_values(|mut app| {
            for _ in 0..steps {
                app.update();
            }
        });
}
