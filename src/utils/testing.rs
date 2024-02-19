use std::{thread, time::Duration};

use bevy::{
    app::{App, FixedUpdate},
    ecs::system::{ResMut, Resource},
    time::{Fixed, Time},
};

/// Steps the Bevy app until FixedUpdate is reached.
/// # Remarks
/// This function can execute any number of Updates and FixedUpdates.
pub(crate) fn step_fixed_update(app: &mut App) {
    // This can not be set to small values like one nanosecond, because it will cause nearly infinite loop.
    const FIXED_DELTA_TIME: f64 = 0.01;
    const MAX_WAIT_STEPS: usize = 32;

    #[derive(Resource)]
    struct FixedUpdateReacherResource {
        reached: bool,
    }

    fn fixed_update_reacher(mut resource: ResMut<FixedUpdateReacherResource>) {
        resource.reached = true;
    }

    app.add_systems(FixedUpdate, fixed_update_reacher)
        .insert_resource(FixedUpdateReacherResource { reached: false })
        .insert_resource(Time::<Fixed>::from_seconds(FIXED_DELTA_TIME));

    for _ in 0..MAX_WAIT_STEPS {
        app.update();

        if app
            .world
            .get_resource::<FixedUpdateReacherResource>()
            .unwrap()
            .reached
        {
            return;
        }

        thread::sleep(Duration::from_secs_f64(FIXED_DELTA_TIME))
    }

    // Do not wait for test break due to timeout if FixedUpdate is not reachable.
    panic!("FixedUpdate did not reach in {MAX_WAIT_STEPS} steps.");
}
