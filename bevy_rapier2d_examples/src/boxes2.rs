use crate::{cleanup_resource, ExampleResource, Examples};

use bevy::prelude::*;
use bevy_rapier2d::prelude::*;

#[derive(Default)]
pub struct ExamplePluginBoxes2;

impl Plugin for ExamplePluginBoxes2 {
    fn build(&self, app: &mut App) {
        app.add_systems(
            OnEnter(Examples::Boxes2),
            (Self::setup_graphics, Self::setup_physics),
        )
        .add_systems(OnExit(Examples::Boxes2), cleanup_resource);
    }
}

impl ExamplePluginBoxes2 {
    fn setup_graphics(mut commands: Commands, mut res: ResMut<ExampleResource>) {
        commands.insert_resource(ClearColor(Color::rgb(
            0xF9 as f32 / 255.0,
            0xF9 as f32 / 255.0,
            0xFF as f32 / 255.0,
        )));

        let camera = commands
            .spawn(Camera2dBundle {
                transform: Transform::from_xyz(0.0, 20.0, 0.0),
                ..default()
            })
            .id();

        res.camera = Some(camera);
    }

    fn setup_physics(mut commands: Commands, mut res: ResMut<ExampleResource>) {
        let root = commands
            .spawn(TransformBundle::default())
            .with_children(|parent| {
                /*
                 * Ground
                 */
                let ground_size = 500.0;
                let ground_height = 10.0;

                parent.spawn((
                    TransformBundle::from(Transform::from_xyz(0.0, 0.0 * -ground_height, 0.0)),
                    Collider::cuboid(ground_size, ground_height),
                ));

                /*
                 * Create the cubes
                 */
                let num = 8;
                let rad = 10.0;

                let shift = rad * 2.0 + rad;
                let centerx = shift * (num / 2) as f32;
                let centery = shift / 2.0;

                let mut offset = -(num as f32) * (rad * 2.0 + rad) * 0.5;

                for j in 0usize..20 {
                    for i in 0..num {
                        let x = i as f32 * shift - centerx + offset;
                        let y = j as f32 * shift + centery + 30.0;

                        parent.spawn((
                            TransformBundle::from(Transform::from_xyz(x, y, 0.0)),
                            RigidBody::Dynamic,
                            Collider::cuboid(rad, rad),
                        ));
                    }

                    offset -= 0.05 * rad * (num as f32 - 1.0);
                }
            })
            .id();

        res.root = Some(root);
    }
}
