use bevy::prelude::*;
use crate::prelude::*;
use crate::render::prelude::*;
use crate::render::render::PositionWireframeMaterial;

/// Spawn newly added debug positions.
pub fn spawn_debug_positions(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    config: Res<RapierConfiguration>,
    mut materials: ResMut<Assets<PositionWireframeMaterial>>,
    query: Query<(Entity, &RapierDebugPosition), Without<RapierDebugPositionLoaded>>
) {
    for (entity, debug) in query.iter() {
        commands.entity(entity)
            .insert(RapierDebugPositionLoaded)
            .insert(Visible { is_visible: true, is_transparent: true })
            .with_children(|parent| {
                parent.spawn()
                .insert(Name::new("Hilt Position"))
                .insert(RapierDebugRenderPosition)
                .insert_bundle(RapierDebugPositionBundle {
                    size: RapierDebugPositionSize(debug.size),
                    mesh: meshes.add(crate::render::mesh::position_mesh(debug, &config)),
                    material: materials.add(PositionWireframeMaterial::default()),
                    ..Default::default()
                });
            });
    }
}
