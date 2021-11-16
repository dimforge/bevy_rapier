use bevy::prelude::*;
use bevy::render::pipeline::PrimitiveTopology;
use crate::prelude::*;
use crate::render::entities::*;
use crate::render::render::WireframeMaterial;

/// Spawn newly added debug paths.
pub fn spawn_debug_paths(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    config: Res<RapierConfiguration>,
    mut materials: ResMut<Assets<WireframeMaterial>>,
    query: Query<(Entity, Option<&ColliderPosition>, Option<&RigidBodyPosition>, &RapierDebugPath), Without<RapierDebugPathLoaded>>
) {
    for (entity, co_pos, rb_pos, debug) in query.iter() {
        let track_path = commands.spawn()
            .insert(RapierDebugRenderPath)
            .insert_bundle(RapierDebugPathWireframeBundle {
                mesh: meshes.add(generate_path_mesh(co_pos, rb_pos, &config)),
                material: materials.add(WireframeMaterial {
                    color: debug.color,
                    dashed: debug.dashed
                }),
                ..Default::default()
            })
            .id();
        commands.entity(entity)
            .insert(RapierDebugPathLoaded(track_path));
    }
}

// Create mesh from an initial position.
fn generate_path_mesh(co: Option<&ColliderPosition>, rb: Option<&RigidBodyPosition>, config: &RapierConfiguration) -> Mesh {
    let mut mesh = Mesh::new(PrimitiveTopology::LineStrip);
    let mut positions = vec![];
    if let Some(pos) = co {
        #[cfg(feature = "dim3")]
        positions.push([pos.translation.x * config.scale, pos.translation.y * config.scale, pos.translation.z * config.scale]);
        #[cfg(feature = "dim2")]
        positions.push([pos.translation.x * config.scale, pos.translation.y * config.scale]);
    } else if let Some(pos) = rb {
        #[cfg(feature = "dim3")]
        positions.push([pos.position.translation.x * config.scale, pos.position.translation.y * config.scale, pos.position.translation.z * config.scale]);
        #[cfg(feature = "dim2")]
        positions.push([pos.position.translation.x * config.scale, pos.position.translation.y * config.scale]);
    }
    mesh.set_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    mesh
}

/// Update the mesh's used by DebugPaths to with a new value.
// TODO: Only update the path if it is greater than some radius.
pub fn update_path_mesh(
    mut meshes: ResMut<Assets<Mesh>>,
    config: Res<RapierConfiguration>,
    query: Query<
        (Entity, &RapierDebugPath, &RapierDebugPathLoaded, Option<&ColliderPosition>, Option<&RigidBodyPosition>),
    >,
    paths_query: Query<&Handle<Mesh>, With<RapierDebugRenderPath>>
) {
    for (_entity, path, debug, co, rb) in query.iter() {
        let handle = paths_query.get(debug.0).unwrap();
        let mesh = meshes.get_mut(handle.clone()).unwrap();
        if let Some(pos) = co {
            #[cfg(feature = "dim3")]
            if let Some(bevy::render::mesh::VertexAttributeValues::Float3(attr)) = mesh.attribute_mut(Mesh::ATTRIBUTE_POSITION) {
                let translation: [f32 ; 3] = (Vec3::from(pos.translation) * Vec3::splat(config.scale)).into();
                if let Some(last) = attr.last() {
                    if last != &translation {
                        if attr.len() == path.length {
                            attr.remove(0);
                        }
                        attr.push(translation);
                    }
                }
            }
            #[cfg(feature = "dim2")]
            if let Some(bevy::render::mesh::VertexAttributeValues::Float2(attr)) = mesh.attribute_mut(Mesh::ATTRIBUTE_POSITION) {
                let translation: [f32 ; 2] = (Vec2::from(pos.translation) * Vec2::new(config.scale, config.scale)).into();
                if let Some(last) = attr.last() {
                    if last != &translation {
                        if attr.len() == path.length {
                            attr.remove(0);
                        }
                        attr.push(translation);
                    }
                }
            }
        } else if let Some(pos) = rb {
            #[cfg(feature = "dim3")]
            if let Some(bevy::render::mesh::VertexAttributeValues::Float3(attr)) = mesh.attribute_mut(Mesh::ATTRIBUTE_POSITION) {
                if attr.len() == path.length {
                    attr.remove(0);
                }
                attr.push((Vec3::from(pos.position.translation) * Vec3::splat(config.scale)).into());
            }
            #[cfg(feature = "dim2")]
            if let Some(bevy::render::mesh::VertexAttributeValues::Float2(attr)) = mesh.attribute_mut(Mesh::ATTRIBUTE_POSITION) {
                if attr.len() == path.length {
                    attr.remove(0);
                }
                attr.push((Vec2::from(pos.position.translation) * Vec2::splat(config.scale)).into());
            }
        }
    }
}
