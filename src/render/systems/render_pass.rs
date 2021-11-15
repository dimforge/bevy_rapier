use bevy::prelude::*;
use bevy::render::render_graph::base::MainPass;
use crate::render::prelude::*;
use crate::render::render::RapierDebugPass;

/// Handles the RapierDebugToggleRenderPass event.
pub fn toggle_render_pass(
    mut commands: Commands,
    mut events: EventReader<RapierDebugToggleRenderPass>,
    mut collider_query: Query<(Entity, Option<&MainPass>, Option<&RapierDebugPass>), (
        With<RapierDebugRenderCollider>,
        Without<RapierDebugRenderPosition>,
        Without<RapierDebugRenderPath>
    )>,
    mut positions_query: Query<(Entity, Option<&MainPass>, Option<&RapierDebugPass>), (
        With<RapierDebugRenderPosition>,
        Without<RapierDebugRenderPath>,
        Without<RapierDebugRenderCollider>
    )>,
    mut paths_query: Query<(Entity, Option<&MainPass>, Option<&RapierDebugPass>), (
        With<RapierDebugRenderPath>,
        Without<RapierDebugRenderCollider>,
        Without<RapierDebugRenderPosition>
    )>,
) {
    for event in events.iter() {
        debug!("toggeling hilt render pass on entities: {:?}", event);
        match event.0 {
            RapierDebugEntities::All => {
                for (entity, main_pass, hilt_pass) in collider_query.iter_mut() {
                    if hilt_pass.is_some() && main_pass.is_none() {
                        commands.entity(entity)
                            .remove::<RapierDebugPass>()
                            .insert(MainPass);
                    } else if hilt_pass.is_none() && main_pass.is_some() {
                        commands.entity(entity)
                            .remove::<MainPass>()
                            .insert(RapierDebugPass);
                    }
                }
                for (entity, main_pass, hilt_pass) in positions_query.iter_mut() {
                    if hilt_pass.is_some() && main_pass.is_none() {
                        commands.entity(entity)
                            .remove::<RapierDebugPass>()
                            .insert(MainPass);
                    } else if hilt_pass.is_none() && main_pass.is_some() {
                        commands.entity(entity)
                            .remove::<MainPass>()
                            .insert(RapierDebugPass);
                    }
                }
                for (entity, main_pass, hilt_pass) in paths_query.iter_mut() {
                    if hilt_pass.is_some() && main_pass.is_none() {
                        commands.entity(entity)
                            .remove::<RapierDebugPass>()
                            .insert(MainPass);
                    } else if hilt_pass.is_none() && main_pass.is_some() {
                        commands.entity(entity)
                            .remove::<MainPass>()
                            .insert(RapierDebugPass);
                    }
                }
            },
            RapierDebugEntities::Colliders => {
                for (entity, main_pass, hilt_pass) in collider_query.iter_mut() {
                    if hilt_pass.is_some() && main_pass.is_none() {
                        commands.entity(entity)
                            .remove::<RapierDebugPass>()
                            .insert(MainPass);
                    } else if hilt_pass.is_none() && main_pass.is_some() {
                        commands.entity(entity)
                            .remove::<MainPass>()
                            .insert(RapierDebugPass);
                    }
                }
            },
            RapierDebugEntities::Positions => {
                for (entity, main_pass, hilt_pass) in positions_query.iter_mut() {
                    if hilt_pass.is_some() && main_pass.is_none() {
                        commands.entity(entity)
                            .remove::<RapierDebugPass>()
                            .insert(MainPass);
                    } else if hilt_pass.is_none() && main_pass.is_some() {
                        commands.entity(entity)
                            .remove::<MainPass>()
                            .insert(RapierDebugPass);
                    }
                }
            },
            RapierDebugEntities::Path => {
                for (entity, main_pass, hilt_pass) in paths_query.iter_mut() {
                    if hilt_pass.is_some() && main_pass.is_none() {
                        commands.entity(entity)
                            .remove::<RapierDebugPass>()
                            .insert(MainPass);
                    } else if hilt_pass.is_none() && main_pass.is_some() {
                        commands.entity(entity)
                            .remove::<MainPass>()
                            .insert(RapierDebugPass);
                    }
                }
            }
        }
    }
}
