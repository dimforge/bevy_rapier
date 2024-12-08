//! Adapted from https://github.com/bevyengine/bevy/blob/5a0c09d38fd63383281991da9d6c353ad7228f50/tests/ecs/ambiguity_detection.rs
//!
//! A test to confirm that `bevy_rapier` doesn't regress its system ambiguities count when using [`DefaultPlugins`].
//! This is run in CI.
//!
//! Note that because this test requires rendering, it isn't actually an integration test!
//! Instead, it's secretly an example: you can run this test manually using `cargo run --example ambiguity_detection`.

use bevy::{
    ecs::schedule::{InternedScheduleLabel, LogLevel, ScheduleBuildSettings},
    prelude::*,
    utils::HashMap,
};
use bevy_rapier3d::{
    plugin::{NoUserData, RapierPhysicsPlugin},
    render::RapierDebugRenderPlugin,
};

fn main() {
    let mut app = App::new();
    app.add_plugins(DefaultPlugins);
    app.add_plugins(RapierPhysicsPlugin::<NoUserData>::default());
    app.add_plugins(RapierDebugRenderPlugin::default());

    let main_app = app.main_mut();
    configure_ambiguity_detection(main_app);

    app.finish();
    app.cleanup();
    app.update();

    let main_app_ambiguities = count_ambiguities(app.main());
    assert_eq!(
        main_app_ambiguities.total(),
        0,
        "Main app has unexpected ambiguities among the following schedules: \n{main_app_ambiguities:#?}.",
    );
}

/// Contains the number of conflicting systems per schedule.
#[derive(Debug, Deref, DerefMut)]
struct AmbiguitiesCount(pub HashMap<InternedScheduleLabel, usize>);

impl AmbiguitiesCount {
    fn total(&self) -> usize {
        self.values().sum()
    }
}

fn configure_ambiguity_detection(sub_app: &mut SubApp) {
    let mut schedules = sub_app.world_mut().resource_mut::<Schedules>();
    for (_, schedule) in schedules.iter_mut() {
        schedule.set_build_settings(ScheduleBuildSettings {
            // NOTE: you can change this to `LogLevel::Ignore` to easily see the current number of ambiguities.
            ambiguity_detection: LogLevel::Warn,
            use_shortnames: false,
            ..default()
        });
    }
}

/// Returns the number of conflicting systems per schedule.
fn count_ambiguities(sub_app: &SubApp) -> AmbiguitiesCount {
    let schedules = sub_app.world().resource::<Schedules>();
    let mut ambiguities = HashMap::new();
    for (_, schedule) in schedules.iter() {
        let ambiguities_in_schedule = schedule.graph().conflicting_systems().len();
        ambiguities.insert(schedule.label(), ambiguities_in_schedule);
    }
    AmbiguitiesCount(ambiguities)
}
