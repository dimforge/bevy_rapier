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
use criterion::{measurement::Measurement, BatchSize, Bencher};

pub fn bench_app<M: Measurement>(
    bencher: &mut Bencher<'_, M>,
    steps: u32,
    setup: impl Fn(&mut App),
) {
    bencher.iter_batched_ref(
        move || {
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

            setup(&mut app);

            while app.plugins_state() != PluginsState::Ready {
                bevy::tasks::tick_global_task_pools_on_main_thread();
            }

            app.finish();
            app.cleanup();
            app
        },
        move |app| {
            for _ in 0..steps {
                app.update();
            }
        },
        BatchSize::PerIteration,
    );
}
