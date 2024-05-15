use crate::dynamics::ReadMassProperties;
use crate::geometry::Collider;
use crate::plugin::{RapierConfiguration, RapierContext};
use crate::prelude::{
    ActiveCollisionTypes, ActiveEvents, ActiveHooks, ColliderDisabled, ColliderMassProperties,
    ColliderScale, CollisionGroups, ContactForceEventThreshold, ContactSkin, Friction,
    MassModifiedEvent, MassProperties, RapierColliderHandle, RapierRigidBodyHandle, Restitution,
    Sensor, SolverGroups,
};
use crate::utils;
use bevy::prelude::*;
use rapier::dynamics::RigidBodyHandle;
use rapier::geometry::ColliderBuilder;
#[cfg(all(feature = "dim3", feature = "async-collider"))]
use {
    crate::prelude::{AsyncCollider, AsyncSceneCollider},
    bevy::scene::SceneInstance,
};

#[cfg(feature = "dim2")]
use bevy::math::Vec3Swizzles;

/// Components related to colliders.
pub type ColliderComponents<'a> = (
    Entity,
    &'a Collider,
    Option<&'a Sensor>,
    Option<&'a ColliderMassProperties>,
    Option<&'a ActiveEvents>,
    Option<&'a ActiveHooks>,
    Option<&'a ActiveCollisionTypes>,
    Option<&'a Friction>,
    Option<&'a Restitution>,
    Option<&'a ContactSkin>,
    Option<&'a CollisionGroups>,
    Option<&'a SolverGroups>,
    Option<&'a ContactForceEventThreshold>,
    Option<&'a ColliderDisabled>,
);

/// System responsible for applying [`GlobalTransform::scale`] and/or [`ColliderScale`] to
/// colliders.
pub fn apply_scale(
    config: Res<RapierConfiguration>,
    mut changed_collider_scales: Query<
        (&mut Collider, &GlobalTransform, Option<&ColliderScale>),
        Or<(
            Changed<Collider>,
            Changed<GlobalTransform>,
            Changed<ColliderScale>,
        )>,
    >,
) {
    for (mut shape, transform, custom_scale) in changed_collider_scales.iter_mut() {
        #[cfg(feature = "dim2")]
        let effective_scale = match custom_scale {
            Some(ColliderScale::Absolute(scale)) => *scale,
            Some(ColliderScale::Relative(scale)) => {
                *scale * transform.compute_transform().scale.xy()
            }
            None => transform.compute_transform().scale.xy(),
        };
        #[cfg(feature = "dim3")]
        let effective_scale = match custom_scale {
            Some(ColliderScale::Absolute(scale)) => *scale,
            Some(ColliderScale::Relative(scale)) => *scale * transform.compute_transform().scale,
            None => transform.compute_transform().scale,
        };

        if shape.scale != crate::geometry::get_snapped_scale(effective_scale) {
            shape.set_scale(effective_scale, config.scaled_shape_subdivision);
        }
    }
}

/// System responsible for applying changes the user made to a collider-related component.
pub fn apply_collider_user_changes(
    mut context: ResMut<RapierContext>,
    config: Res<RapierConfiguration>,
    (changed_collider_transforms, parent_query, transform_query): (
        Query<
            (Entity, &RapierColliderHandle, &GlobalTransform),
            (Without<RapierRigidBodyHandle>, Changed<GlobalTransform>),
        >,
        Query<&Parent>,
        Query<&Transform>,
    ),

    changed_shapes: Query<(&RapierColliderHandle, &Collider), Changed<Collider>>,
    changed_active_events: Query<(&RapierColliderHandle, &ActiveEvents), Changed<ActiveEvents>>,
    changed_active_hooks: Query<(&RapierColliderHandle, &ActiveHooks), Changed<ActiveHooks>>,
    changed_active_collision_types: Query<
        (&RapierColliderHandle, &ActiveCollisionTypes),
        Changed<ActiveCollisionTypes>,
    >,
    (changed_friction, changed_restitution, changed_contact_skin): (
        Query<(&RapierColliderHandle, &Friction), Changed<Friction>>,
        Query<(&RapierColliderHandle, &Restitution), Changed<Restitution>>,
        Query<(&RapierColliderHandle, &ContactSkin), Changed<ContactSkin>>,
    ),
    changed_collision_groups: Query<
        (&RapierColliderHandle, &CollisionGroups),
        Changed<CollisionGroups>,
    >,
    changed_solver_groups: Query<(&RapierColliderHandle, &SolverGroups), Changed<SolverGroups>>,
    changed_sensors: Query<(&RapierColliderHandle, &Sensor), Changed<Sensor>>,
    changed_disabled: Query<(&RapierColliderHandle, &ColliderDisabled), Changed<ColliderDisabled>>,
    changed_contact_force_threshold: Query<
        (&RapierColliderHandle, &ContactForceEventThreshold),
        Changed<ContactForceEventThreshold>,
    >,
    changed_collider_mass_props: Query<
        (&RapierColliderHandle, &ColliderMassProperties),
        Changed<ColliderMassProperties>,
    >,

    mut mass_modified: EventWriter<MassModifiedEvent>,
) {
    for (entity, handle, transform) in changed_collider_transforms.iter() {
        if context.collider_parent(entity).is_some() {
            let (_, collider_position) =
                collider_offset(entity, &context, &parent_query, &transform_query);

            if let Some(co) = context.colliders.get_mut(handle.0) {
                co.set_position_wrt_parent(utils::transform_to_iso(&collider_position));
            }
        } else if let Some(co) = context.colliders.get_mut(handle.0) {
            co.set_position(utils::transform_to_iso(&transform.compute_transform()))
        }
    }

    for (handle, shape) in changed_shapes.iter() {
        if let Some(co) = context.colliders.get_mut(handle.0) {
            let mut scaled_shape = shape.clone();
            scaled_shape.set_scale(shape.scale, config.scaled_shape_subdivision);
            co.set_shape(scaled_shape.raw.clone());

            if let Some(body) = co.parent() {
                if let Some(body_entity) = context.rigid_body_entity(body) {
                    mass_modified.send(body_entity.into());
                }
            }
        }
    }

    for (handle, active_events) in changed_active_events.iter() {
        if let Some(co) = context.colliders.get_mut(handle.0) {
            co.set_active_events((*active_events).into())
        }
    }

    for (handle, active_hooks) in changed_active_hooks.iter() {
        if let Some(co) = context.colliders.get_mut(handle.0) {
            co.set_active_hooks((*active_hooks).into())
        }
    }

    for (handle, active_collision_types) in changed_active_collision_types.iter() {
        if let Some(co) = context.colliders.get_mut(handle.0) {
            co.set_active_collision_types((*active_collision_types).into())
        }
    }

    for (handle, friction) in changed_friction.iter() {
        if let Some(co) = context.colliders.get_mut(handle.0) {
            co.set_friction(friction.coefficient);
            co.set_friction_combine_rule(friction.combine_rule.into());
        }
    }

    for (handle, restitution) in changed_restitution.iter() {
        if let Some(co) = context.colliders.get_mut(handle.0) {
            co.set_restitution(restitution.coefficient);
            co.set_restitution_combine_rule(restitution.combine_rule.into());
        }
    }

    for (handle, contact_skin) in changed_contact_skin.iter() {
        if let Some(co) = context.colliders.get_mut(handle.0) {
            co.set_contact_skin(contact_skin.0);
        }
    }

    for (handle, collision_groups) in changed_collision_groups.iter() {
        if let Some(co) = context.colliders.get_mut(handle.0) {
            co.set_collision_groups((*collision_groups).into());
        }
    }

    for (handle, solver_groups) in changed_solver_groups.iter() {
        if let Some(co) = context.colliders.get_mut(handle.0) {
            co.set_solver_groups((*solver_groups).into());
        }
    }

    for (handle, _) in changed_sensors.iter() {
        if let Some(co) = context.colliders.get_mut(handle.0) {
            co.set_sensor(true);
        }
    }

    for (handle, _) in changed_disabled.iter() {
        if let Some(co) = context.colliders.get_mut(handle.0) {
            co.set_enabled(false);
        }
    }

    for (handle, threshold) in changed_contact_force_threshold.iter() {
        if let Some(co) = context.colliders.get_mut(handle.0) {
            co.set_contact_force_event_threshold(threshold.0);
        }
    }

    for (handle, mprops) in changed_collider_mass_props.iter() {
        if let Some(co) = context.colliders.get_mut(handle.0) {
            match mprops {
                ColliderMassProperties::Density(density) => co.set_density(*density),
                ColliderMassProperties::Mass(mass) => co.set_mass(*mass),
                ColliderMassProperties::MassProperties(mprops) => {
                    co.set_mass_properties(mprops.into_rapier())
                }
            }

            if let Some(body) = co.parent() {
                if let Some(body_entity) = context.rigid_body_entity(body) {
                    mass_modified.send(body_entity.into());
                }
            }
        }
    }
}

pub(crate) fn collider_offset(
    entity: Entity,
    context: &RapierContext,
    parent_query: &Query<&Parent>,
    transform_query: &Query<&Transform>,
) -> (Option<RigidBodyHandle>, Transform) {
    let mut body_entity = entity;
    let mut body_handle = context.entity2body.get(&body_entity).copied();
    let mut child_transform = Transform::default();
    while body_handle.is_none() {
        if let Ok(parent_entity) = parent_query.get(body_entity) {
            if let Ok(transform) = transform_query.get(body_entity) {
                child_transform = *transform * child_transform;
            }
            body_entity = parent_entity.get();
        } else {
            break;
        }

        body_handle = context.entity2body.get(&body_entity).copied();
    }

    if body_handle.is_some() {
        if let Ok(transform) = transform_query.get(body_entity) {
            let scale_transform = Transform {
                scale: transform.scale,
                ..default()
            };

            child_transform = scale_transform * child_transform;
        }
    }

    (body_handle, child_transform)
}

/// System responsible for creating new Rapier colliders from the related `bevy_rapier` components.
pub fn init_colliders(
    mut commands: Commands,
    config: Res<RapierConfiguration>,
    mut context: ResMut<RapierContext>,
    colliders: Query<(ColliderComponents, Option<&GlobalTransform>), Without<RapierColliderHandle>>,
    mut rigid_body_mprops: Query<&mut ReadMassProperties>,
    parent_query: Query<&Parent>,
    transform_query: Query<&Transform>,
) {
    let context = &mut *context;

    for (
        (
            entity,
            shape,
            sensor,
            mprops,
            active_events,
            active_hooks,
            active_collision_types,
            friction,
            restitution,
            contact_skin,
            collision_groups,
            solver_groups,
            contact_force_event_threshold,
            disabled,
        ),
        global_transform,
    ) in colliders.iter()
    {
        let mut scaled_shape = shape.clone();
        scaled_shape.set_scale(shape.scale, config.scaled_shape_subdivision);
        let mut builder = ColliderBuilder::new(scaled_shape.raw.clone());

        builder = builder.sensor(sensor.is_some());
        builder = builder.enabled(disabled.is_none());

        if let Some(mprops) = mprops {
            builder = match mprops {
                ColliderMassProperties::Density(density) => builder.density(*density),
                ColliderMassProperties::Mass(mass) => builder.mass(*mass),
                ColliderMassProperties::MassProperties(mprops) => {
                    builder.mass_properties(mprops.into_rapier())
                }
            };
        }

        if let Some(active_events) = active_events {
            builder = builder.active_events((*active_events).into());
        }

        if let Some(active_hooks) = active_hooks {
            builder = builder.active_hooks((*active_hooks).into());
        }

        if let Some(active_collision_types) = active_collision_types {
            builder = builder.active_collision_types((*active_collision_types).into());
        }

        if let Some(friction) = friction {
            builder = builder
                .friction(friction.coefficient)
                .friction_combine_rule(friction.combine_rule.into());
        }

        if let Some(restitution) = restitution {
            builder = builder
                .restitution(restitution.coefficient)
                .restitution_combine_rule(restitution.combine_rule.into());
        }

        if let Some(contact_skin) = contact_skin {
            builder = builder.contact_skin(contact_skin.0);
        }

        if let Some(collision_groups) = collision_groups {
            builder = builder.collision_groups((*collision_groups).into());
        }

        if let Some(solver_groups) = solver_groups {
            builder = builder.solver_groups((*solver_groups).into());
        }

        if let Some(threshold) = contact_force_event_threshold {
            builder = builder.contact_force_event_threshold(threshold.0);
        }

        let body_entity = entity;
        let (body_handle, child_transform) =
            collider_offset(entity, context, &parent_query, &transform_query);

        builder = builder.user_data(entity.to_bits() as u128);

        let handle = if let Some(body_handle) = body_handle {
            builder = builder.position(utils::transform_to_iso(&child_transform));
            let handle =
                context
                    .colliders
                    .insert_with_parent(builder, body_handle, &mut context.bodies);
            if let Ok(mut mprops) = rigid_body_mprops.get_mut(body_entity) {
                // Inserting the collider changed the rigid-body’s mass properties.
                // Read them back from the engine.
                if let Some(parent_body) = context.bodies.get(body_handle) {
                    mprops.set(MassProperties::from_rapier(
                        parent_body.mass_properties().local_mprops,
                    ));
                }
            }
            handle
        } else {
            let global_transform = global_transform.cloned().unwrap_or_default();
            builder = builder.position(utils::transform_to_iso(
                &global_transform.compute_transform(),
            ));
            context.colliders.insert(builder)
        };

        commands.entity(entity).insert(RapierColliderHandle(handle));
        context.entity2collider.insert(entity, handle);
    }
}

/// System responsible for creating `Collider` components from `AsyncCollider` components if the
/// corresponding mesh has become available.
#[cfg(all(feature = "dim3", feature = "async-collider"))]
pub fn init_async_colliders(
    mut commands: Commands,
    meshes: Res<Assets<Mesh>>,
    async_colliders: Query<(Entity, &Handle<Mesh>, &AsyncCollider)>,
) {
    for (entity, mesh_handle, async_collider) in async_colliders.iter() {
        if let Some(mesh) = meshes.get(mesh_handle) {
            match Collider::from_bevy_mesh(mesh, &async_collider.0) {
                Some(collider) => {
                    commands
                        .entity(entity)
                        .insert(collider)
                        .remove::<AsyncCollider>();
                }
                None => error!("Unable to generate collider from mesh {:?}", mesh),
            }
        }
    }
}

/// System responsible for creating `Collider` components from `AsyncSceneCollider` components if the
/// corresponding scene has become available.
#[cfg(all(feature = "dim3", feature = "async-collider"))]
pub fn init_async_scene_colliders(
    mut commands: Commands,
    meshes: Res<Assets<Mesh>>,
    scene_spawner: Res<SceneSpawner>,
    async_colliders: Query<(Entity, &SceneInstance, &AsyncSceneCollider)>,
    children: Query<&Children>,
    mesh_handles: Query<(&Name, &Handle<Mesh>)>,
) {
    for (scene_entity, scene_instance, async_collider) in async_colliders.iter() {
        if scene_spawner.instance_is_ready(**scene_instance) {
            for child_entity in children.iter_descendants(scene_entity) {
                if let Ok((name, handle)) = mesh_handles.get(child_entity) {
                    let shape = async_collider
                        .named_shapes
                        .get(name.as_str())
                        .unwrap_or(&async_collider.shape);
                    if let Some(shape) = shape {
                        let mesh = meshes.get(handle).unwrap(); // NOTE: Mesh is already loaded
                        match Collider::from_bevy_mesh(mesh, shape) {
                            Some(collider) => {
                                commands.entity(child_entity).insert(collider);
                            }
                            None => error!(
                                "Unable to generate collider from mesh {:?} with name {}",
                                mesh, name
                            ),
                        }
                    }
                }
            }

            commands.entity(scene_entity).remove::<AsyncSceneCollider>();
        }
    }
}

#[cfg(test)]
pub mod test {
    #[test]
    #[cfg(all(feature = "dim3", feature = "async-collider"))]
    fn async_collider_initializes() {
        use super::*;
        use crate::plugin::systems::tests::HeadlessRenderPlugin;

        let mut app = App::new();
        app.add_plugins(HeadlessRenderPlugin)
            .add_systems(Update, init_async_colliders);

        let mut meshes = app.world.resource_mut::<Assets<Mesh>>();
        let cube = meshes.add(Cuboid::default());

        let entity = app.world.spawn((cube, AsyncCollider::default())).id();

        app.update();

        let entity = app.world.entity(entity);
        assert!(
            entity.get::<Collider>().is_some(),
            "Collider component should be added"
        );
        assert!(
            entity.get::<AsyncCollider>().is_none(),
            "AsyncCollider component should be removed after Collider component creation"
        );
    }

    #[test]
    #[cfg(all(feature = "dim3", feature = "async-collider"))]
    fn async_scene_collider_initializes() {
        use super::*;
        use crate::plugin::systems::tests::HeadlessRenderPlugin;

        let mut app = App::new();
        app.add_plugins(HeadlessRenderPlugin)
            .add_systems(PostUpdate, init_async_scene_colliders);

        let mut meshes = app.world.resource_mut::<Assets<Mesh>>();
        let cube_handle = meshes.add(Cuboid::default());
        let capsule_handle = meshes.add(Capsule3d::default());
        let cube = app.world.spawn((Name::new("Cube"), cube_handle)).id();
        let capsule = app.world.spawn((Name::new("Capsule"), capsule_handle)).id();

        let mut scenes = app.world.resource_mut::<Assets<Scene>>();
        let scene = scenes.add(Scene::new(World::new()));

        let mut named_shapes = bevy::utils::HashMap::new();
        named_shapes.insert("Capsule".to_string(), None);
        let parent = app
            .world
            .spawn((
                scene,
                AsyncSceneCollider {
                    named_shapes,
                    ..Default::default()
                },
            ))
            .push_children(&[cube, capsule])
            .id();

        app.update();

        assert!(
            app.world.entity(cube).get::<Collider>().is_some(),
            "Collider component should be added for cube"
        );
        assert!(
            app.world.entity(capsule).get::<Collider>().is_none(),
            "Collider component shouldn't be added for capsule"
        );
        assert!(
            app.world.entity(parent).get::<AsyncCollider>().is_none(),
            "AsyncSceneCollider component should be removed after Collider components creation"
        );
    }
}
