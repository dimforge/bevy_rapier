use bevy::prelude::*;
use crate::render::prelude::*;

/// Handle the RapierDebugToggleVisibility event.
pub fn toggle_visibility(
    mut events: EventReader<RapierDebugToggleVisibility>,
    mut collider_query: Query<&mut Visible, (
        With<RapierDebugRenderCollider>,
        Without<RapierDebugRenderPosition>,
        Without<RapierDebugRenderPath>
    )>,
    mut positions_query: Query<&mut Visible, (
        With<RapierDebugRenderPosition>,
        Without<RapierDebugRenderCollider>,
        Without<RapierDebugRenderPath>
    )>,
    mut paths_query: Query<&mut Visible, (
        With<RapierDebugRenderPath>,
        Without<RapierDebugRenderCollider>,
        Without<RapierDebugPosition>
    )>
) {
    for event in events.iter() {
        debug!("toggeling visibility on entities: {:?}", event);
        match event.0 {
            RapierDebugEntities::All => {
                for mut visible in collider_query.iter_mut() {
                    visible.is_visible = !visible.is_visible;
                }
                for mut visible in positions_query.iter_mut() {
                    visible.is_visible = !visible.is_visible;
                }
                for mut visible in paths_query.iter_mut() {
                    visible.is_visible = !visible.is_visible;
                }
            },
            RapierDebugEntities::Colliders => {
                for mut visible in collider_query.iter_mut() {
                    visible.is_visible = !visible.is_visible;
                }
            },
            RapierDebugEntities::Positions => {
                for mut visible in positions_query.iter_mut() {
                    visible.is_visible = !visible.is_visible;
                }
            },
            RapierDebugEntities::Path => {
                for mut visible in paths_query.iter_mut() {
                    visible.is_visible = !visible.is_visible;
                }
            }
        }
    }
}
