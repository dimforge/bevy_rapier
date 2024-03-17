#[cfg(feature = "dim2")]
use bevy::math::Vec3Swizzles;

pub use character_controller::*;
pub use collider::*;
pub use joints::*;
pub use removals::*;
pub use rigid_body::*;
pub use simulate::*;
pub use writeback::*;

mod character_controller;
mod collider;
mod joints;
mod removals;
mod rigid_body;
mod simulate;
mod writeback;

#[cfg(test)]
mod tests {
    #[cfg(all(feature = "dim3", feature = "async-collider"))]
    use bevy::prelude::{Capsule3d, Cuboid};
    use bevy::{
        asset::AssetPlugin,
        ecs::event::Events,
        render::{
            settings::{RenderCreation, WgpuSettings},
            RenderPlugin,
        },
        scene::ScenePlugin,
        time::TimePlugin,
        window::WindowPlugin,
    };
    use std::f32::consts::PI;

    use super::*;
    use crate::plugin::{NoUserData, RapierPhysicsPlugin};

    #[test]
    fn colliding_entities_updates() {
        let mut app = App::new();
        app.add_event::<CollisionEvent>()
            .add_systems(Update, update_colliding_entities);

        let entity1 = app.world.spawn(CollidingEntities::default()).id();
        let entity2 = app.world.spawn(CollidingEntities::default()).id();

        let mut collision_events = app
            .world
            .get_resource_mut::<Events<CollisionEvent>>()
            .unwrap();
        collision_events.send(CollisionEvent::Started(
            entity1,
            entity2,
            CollisionEventFlags::SENSOR,
        ));

        app.update();

        let colliding_entities1 = app
            .world
            .entity(entity1)
            .get::<CollidingEntities>()
            .unwrap();
        assert_eq!(
            colliding_entities1.len(),
            1,
            "There should be one colliding entity"
        );
        assert_eq!(
            colliding_entities1.iter().next().unwrap(),
            entity2,
            "Colliding entity should be equal to the second entity"
        );

        let colliding_entities2 = app
            .world
            .entity(entity2)
            .get::<CollidingEntities>()
            .unwrap();
        assert_eq!(
            colliding_entities2.len(),
            1,
            "There should be one colliding entity"
        );
        assert_eq!(
            colliding_entities2.iter().next().unwrap(),
            entity1,
            "Colliding entity should be equal to the first entity"
        );

        let mut collision_events = app
            .world
            .get_resource_mut::<Events<CollisionEvent>>()
            .unwrap();
        collision_events.send(CollisionEvent::Stopped(
            entity1,
            entity2,
            CollisionEventFlags::SENSOR,
        ));

        app.update();

        let colliding_entities1 = app
            .world
            .entity(entity1)
            .get::<CollidingEntities>()
            .unwrap();
        assert!(
            colliding_entities1.is_empty(),
            "Colliding entity should be removed from the CollidingEntities component when the collision ends"
        );

        let colliding_entities2 = app
            .world
            .entity(entity2)
            .get::<CollidingEntities>()
            .unwrap();
        assert!(
            colliding_entities2.is_empty(),
            "Colliding entity should be removed from the CollidingEntities component when the collision ends"
        );
    }

    #[test]
    #[cfg(all(feature = "dim3", feature = "async-collider"))]
    fn async_collider_initializes() {
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

    #[test]
    fn transform_propagation() {
        let mut app = App::new();
        app.add_plugins((
            HeadlessRenderPlugin,
            TransformPlugin,
            TimePlugin,
            RapierPhysicsPlugin::<NoUserData>::default(),
        ));

        let zero = (Transform::default(), Transform::default());

        let different = (
            Transform {
                translation: Vec3::X * 10.0,
                rotation: Quat::from_rotation_x(PI),
                ..Default::default()
            },
            Transform {
                translation: Vec3::Y * 10.0,
                rotation: Quat::from_rotation_x(PI),
                ..Default::default()
            },
        );

        let same = (different.0, different.0);

        for (child_transform, parent_transform) in [zero, same, different] {
            let child = app
                .world
                .spawn((
                    TransformBundle::from(child_transform),
                    RigidBody::Fixed,
                    Collider::ball(1.0),
                ))
                .id();

            app.world
                .spawn(TransformBundle::from(parent_transform))
                .push_children(&[child]);

            app.update();

            let child_transform = app.world.entity(child).get::<GlobalTransform>().unwrap();
            let context = app.world.resource::<RapierContext>();
            let child_handle = context.entity2body[&child];
            let child_body = context.bodies.get(child_handle).unwrap();
            let body_transform = utils::iso_to_transform(child_body.position());
            assert_eq!(
                GlobalTransform::from(body_transform),
                *child_transform,
                "Collider transform should have have global rotation and translation"
            );
        }
    }

    #[test]
    fn transform_propagation2() {
        let mut app = App::new();
        app.add_plugins((
            HeadlessRenderPlugin,
            TransformPlugin,
            TimePlugin,
            RapierPhysicsPlugin::<NoUserData>::default(),
        ));

        let zero = (Transform::default(), Transform::default());

        let different = (
            Transform {
                translation: Vec3::X * 10.0,
                // NOTE: in 2D the test will fail if the rotation is wrt. an axis
                //       other than Z because 2D physics objects can’t rotate wrt.
                //       other axes.
                rotation: Quat::from_rotation_z(PI),
                ..Default::default()
            },
            Transform {
                translation: Vec3::Y * 10.0,
                rotation: Quat::from_rotation_z(PI),
                ..Default::default()
            },
        );

        let same = (different.0, different.0);

        for (child_transform, parent_transform) in [zero, same, different] {
            let child = app
                .world
                .spawn((TransformBundle::from(child_transform), Collider::ball(1.0)))
                .id();

            let parent = app
                .world
                .spawn((TransformBundle::from(parent_transform), RigidBody::Fixed))
                .push_children(&[child])
                .id();

            app.update();

            let child_transform = app
                .world
                .entity(child)
                .get::<GlobalTransform>()
                .unwrap()
                .compute_transform();
            let context = app.world.resource::<RapierContext>();
            let parent_handle = context.entity2body[&parent];
            let parent_body = context.bodies.get(parent_handle).unwrap();
            let child_collider_handle = parent_body.colliders()[0];
            let child_collider = context.colliders.get(child_collider_handle).unwrap();
            let body_transform = utils::iso_to_transform(child_collider.position());
            approx::assert_relative_eq!(
                body_transform.translation,
                child_transform.translation,
                epsilon = 1.0e-5
            );

            // Adjust signs to account for the quaternion’s double covering.
            let comparison_child_rotation =
                if body_transform.rotation.w * child_transform.rotation.w < 0.0 {
                    -child_transform.rotation
                } else {
                    child_transform.rotation
                };

            approx::assert_relative_eq!(
                body_transform.rotation,
                comparison_child_rotation,
                epsilon = 1.0e-5
            );
            approx::assert_relative_eq!(body_transform.scale, child_transform.scale,);
        }
    }

    // Allows run tests for systems containing rendering related things without GPU
    struct HeadlessRenderPlugin;

    impl Plugin for HeadlessRenderPlugin {
        fn build(&self, app: &mut App) {
            app.add_plugins((
                WindowPlugin::default(),
                AssetPlugin::default(),
                ScenePlugin::default(),
                RenderPlugin {
                    render_creation: RenderCreation::Automatic(WgpuSettings {
                        backends: None,
                        ..Default::default()
                    }),
                    ..Default::default()
                },
                ImagePlugin::default(),
            ));
        }
    }
}
