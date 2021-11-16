use bevy::prelude::*;
use crate::prelude::*;
use crate::render::prelude::*;
use crate::render::render::WireframeMaterial;

/// Spawn newly added debug colliders.
pub fn spawn_debug_colliders(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    config: Res<RapierConfiguration>,
    mut materials: ResMut<Assets<WireframeMaterial>>,
    query: Query<
        (
            Entity, &ColliderShape, &ColliderType, &RapierDebugCollider, &ColliderPosition,
            Option<&RigidBodyPositionSync>, Option<&ColliderPositionSync>,
            Option<&RigidBodyPosition>
        ),
        Without<RapierDebugColliderLoaded>
    >
) {
    for (entity, shape, ty, debug, co_pos, rb_sync, co_sync, rb_pos) in query.iter() {
        let transform = {
            if co_sync.is_some() {
                Transform::identity()
            } else if rb_sync.is_some() {
                if let Some(rb_pos) = rb_pos {
                    rigid_body_transform(rb_pos, co_pos)
                } else {
                    Default::default()
                }
            } else {
                collider_transform(co_pos)
            }
        };
        commands.entity(entity)
            .insert(RapierDebugColliderLoaded)
            .insert(Visible { is_visible: true, is_transparent: true })
            .with_children(|parent| {
                if let Some(mesh) = collider_to_mesh(shape, &config) {
                    parent.spawn()
                        .insert(Name::new("Hilt Collider"))
                        .insert(RapierDebugRenderCollider)
                        .insert_bundle(RapierDebugColliderWireframeBundle {
                            mesh: meshes.add(mesh),
                            material: materials.add(crate::render::render::WireframeMaterial {
                                color: debug.color,
                                dashed: ty == &ColliderType::Sensor
                            }),
                            global_transform: GlobalTransform::from(transform),
                            transform,
                            ..Default::default()
                        });
                }
            });
    }
}

#[cfg(feature = "dim3")]
fn collider_transform(co_pos: &ColliderPosition) -> Transform {
    let mut transform = Transform::from_translation(co_pos.translation.into());
    transform.rotation = co_pos.rotation.into();
    transform
}

#[cfg(feature = "dim2")]
fn collider_transform(co_pos: &ColliderPosition) -> Transform {
    Transform::from_xyz(
        co_pos.translation.x,
        co_pos.translation.y,
        1.0
    )
}

#[cfg(feature = "dim3")]
fn rigid_body_transform(rb_pos: &RigidBodyPosition, co_pos: &ColliderPosition) -> Transform {
    let mut co_transform = Transform::from_translation(Vec3::from(co_pos.translation) - Vec3::from(rb_pos.position.translation));
    co_transform.rotation = Quat::from(co_pos.rotation);
    co_transform
}

#[cfg(feature = "dim2")]
fn rigid_body_transform(rb_pos: &RigidBodyPosition, co_pos: &ColliderPosition) -> Transform {
    let pos = Vec2::from(co_pos.translation) - Vec2::from(rb_pos.position.translation);
    Transform::from_xyz(pos.x, pos.y, 1.0)
}

fn collider_to_mesh(shape: &ColliderShape, config: &RapierConfiguration) -> Option<Mesh> {
    match shape.shape_type() {
        ShapeType::Cuboid => {
            let cuboid = shape.as_cuboid().unwrap();
            Some(crate::render::mesh::wire_cube(cuboid, config))
        },
        ShapeType::Ball => {
            let ball = shape.as_ball().unwrap();
            Some(crate::render::mesh::wire_sphere(ball.radius * config.scale))
        },
        ShapeType::TriMesh => {
            let trimesh = shape.as_trimesh().unwrap();
            Some(crate::render::mesh::wire_trimesh(trimesh))
        },
        ShapeType::Capsule => {
            let capsule = shape.as_capsule().unwrap();
            Some(crate::render::mesh::wire_capsule(capsule, config))
        },
        ShapeType::Polyline => {
            let polyline = shape.as_polyline().unwrap();
            Some(crate::render::mesh::wire_polyline(polyline))
        },
        ShapeType::Segment => {
            let segment = shape.as_segment().unwrap();
            Some(crate::render::mesh::wire_segment(segment))
        },
        #[cfg(feature = "dim3")]
        ShapeType::Cylinder => {
            let cylinder = shape.as_cylinder().unwrap();
            Some(crate::render::mesh::wire_cylinder(cylinder, config))
        },
        ty => {
            warn!("Unable to render collider shape type: {:?}", ty);
            None
        }
    }
}
