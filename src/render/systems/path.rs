use bevy::prelude::*;
use bevy::render::pipeline::PrimitiveTopology;
use crate::prelude::*;
use crate::render::entities::*;
use crate::render::render::WireframeMaterial;

pub fn spawn_debug_paths(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<WireframeMaterial>>,
    query: Query<(Entity, Option<&ColliderPosition>, Option<&RigidBodyPosition>, &RapierDebugPath), Without<RapierDebugPathLoaded>>
) {
    for (entity, co_pos, rb_pos, debug) in query.iter() {
        let track_path = commands.spawn()
            .insert(RapierDebugRenderPath)
            .insert_bundle(RapierDebugPathWireframeBundle {
                mesh: meshes.add(generate_path_mesh(co_pos, rb_pos)),
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

fn generate_path_mesh(co: Option<&ColliderPosition>, rb: Option<&RigidBodyPosition>) -> Mesh {
    let mut mesh = Mesh::new(PrimitiveTopology::LineStrip);
    let mut positions = vec![];
    if let Some(pos) = co {
        #[cfg(feature = "dim3")]
        positions.push([pos.translation.x, pos.translation.y, pos.translation.z]);
        #[cfg(feature = "dim2")]
        positions.push([pos.translation.x, pos.translation.y]);
    }
    if let Some(pos) = rb {
        #[cfg(feature = "dim3")]
        positions.push([pos.position.translation.x, pos.position.translation.y, pos.position.translation.z]);
        #[cfg(feature = "dim2")]
        positions.push([pos.position.translation.x, pos.position.translation.y]);
    }
    mesh.set_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    mesh
}

pub fn update_path_mesh(
    mut meshes: ResMut<Assets<Mesh>>,
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
                let translation: [f32 ; 3] = Vec3::from(pos.translation).into();
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
                let translation: [f32 ; 2] = Vec2::from(pos.translation).into();
                if let Some(last) = attr.last() {
                    if last != &translation {
                        if attr.len() == path.length {
                            attr.remove(0);
                        }
                        attr.push(translation);
                    }
                }
            }
        }
        if let Some(pos) = rb {
            #[cfg(feature = "dim3")]
            if let Some(bevy::render::mesh::VertexAttributeValues::Float3(attr)) = mesh.attribute_mut(Mesh::ATTRIBUTE_POSITION) {
                if attr.len() == path.length {
                    attr.remove(0);
                }
                attr.push(pos.position.translation.into());
            }
            #[cfg(feature = "dim2")]
            if let Some(bevy::render::mesh::VertexAttributeValues::Float2(attr)) = mesh.attribute_mut(Mesh::ATTRIBUTE_POSITION) {
                if attr.len() == path.length {
                    attr.remove(0);
                }
                attr.push(pos.position.translation.into());
            }
        }
    }
}
