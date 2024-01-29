use std::collections::VecDeque;

use bevy::time::{Fixed, Real, Time};

pub(crate) struct RapierInterpolationContext {
    // Collect lasts delta times across the presentation frames.
    temporal_presentation_delta_time: VecDeque<f32>,
    elapsed_time_since_last_fixed_update: f32,
}

impl RapierInterpolationContext {
    pub fn get_lerp_percentage_for_frame(
        &mut self,
        fixed_time: &Time<Fixed>,
        presentation_time: &Time<Real>,
    ) -> f32 {
        // NOTE: Create little margin in miliseconds which prevents constantly interpolation mode switching e.g. when
        //       fixed delta time is almost equal to presentation delta time.
        const INTERPOLATION_MODE_MARGIN: f32 = 0.001;

        // Smooths out the presentation delta time over the last N frames.
        self.temporal_presentation_delta_time.pop_front();
        self.temporal_presentation_delta_time
            .push_back(presentation_time.delta_seconds());
        let temporal_presentation_time_delta =
            self.temporal_presentation_delta_time.iter().sum::<f32>()
                / self.temporal_presentation_delta_time.len() as f32;

        self.elapsed_time_since_last_fixed_update += presentation_time.delta_seconds();

        // NOTE: Use different interpolation modes depending where fixed delta time is greater than presentation delta
        //       time and otherwise when is not.
        match fixed_time.delta_seconds() + INTERPOLATION_MODE_MARGIN
            >= temporal_presentation_time_delta
        {
            true => fixed_time.overstep_percentage(),
            false => (self.elapsed_time_since_last_fixed_update
                / presentation_time.delta_seconds())
            .min(1.0),
        }
    }

    pub fn notice_fixed_update_frame(&mut self) {
        self.elapsed_time_since_last_fixed_update = 0.0;
    }

    pub fn is_after_update(&self) -> bool {
        self.elapsed_time_since_last_fixed_update > 0.0
    }
}

impl Default for RapierInterpolationContext {
    fn default() -> Self {
        Self {
            temporal_presentation_delta_time: vec![0.0; 8].into(),
            elapsed_time_since_last_fixed_update: 0.0,
        }
    }
}
