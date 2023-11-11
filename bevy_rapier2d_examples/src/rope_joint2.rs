use crate::{cleanup_resource, ExampleResource, Examples};

use bevy::prelude::*;
use bevy_rapier2d::prelude::*;

#[derive(Default)]
pub struct ExamplePluginRopeJoint2;

impl Plugin for ExamplePluginRopeJoint2 {
    fn build(&self, app: &mut App) {
        app.add_systems(
            OnEnter(Examples::RopeJoint2),
            (Self::setup_graphics, Self::setup_physics),
        )
        .add_systems(OnExit(Examples::RopeJoint2), cleanup_resource);
    }
}

impl ExamplePluginRopeJoint2 {
    fn setup_graphics(mut commands: Commands, mut res: ResMut<ExampleResource>) {
        commands.insert_resource(ClearColor(Color::rgb(
            0x00 as f32 / 255.0,
            0xF9 as f32 / 255.0,
            0xFF as f32 / 255.0,
        )));

        let camera = commands
            .spawn(Camera2dBundle {
                transform: Transform::from_xyz(0.0, -200.0, 0.0),
                ..default()
            })
            .id();

        res.camera = Some(camera);
    }

    pub fn setup_physics(mut commands: Commands, mut res: ResMut<ExampleResource>) {
        let root = commands
            .spawn(TransformBundle::default())
            .with_children(|root| {
                let rad = 10.0;
                let rope_length = rad * 10.0;

                let parent = root
                    .spawn((
                        TransformBundle::from(Transform::from_xyz(0.0, 0.0, 0.0)),
                        RigidBody::Fixed,
                        Collider::cuboid(rad, rad),
                    ))
                    .id();

                let joint = RopeJointBuilder::new()
                    .local_anchor2(Vec2::new(rope_length, 0.0))
                    .limits([0.0, rope_length]);

                root.spawn((
                    TransformBundle::from(Transform::from_xyz(-rad * 2.0, 0.0, 0.0)),
                    RigidBody::Dynamic,
                    Collider::cuboid(rad, rad),
                ))
                .insert(ImpulseJoint::new(parent, joint));
            })
            .id();

        res.root = Some(root);
    }
}
