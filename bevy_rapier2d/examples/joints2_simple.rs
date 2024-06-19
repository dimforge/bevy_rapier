use std::num::NonZeroUsize;

use bevy::{
    app::PluginsState, input::common_conditions::input_just_pressed, prelude::*, transform,
};
use bevy_inspector_egui::quick::WorldInspectorPlugin;
use bevy_rapier2d::prelude::*;

fn main() {
    let mut app = App::new();
    app.insert_resource(ClearColor(Color::rgb(
        0xF9 as f32 / 255.0,
        0xF9 as f32 / 255.0,
        0xFF as f32 / 255.0,
    )))
    .add_plugins((
        DefaultPlugins,
        RapierPhysicsPlugin::<NoUserData>::pixels_per_meter(100.0),
        RapierDebugRenderPlugin::default(),
    ))
    .add_plugins(WorldInspectorPlugin::new())
    .add_systems(Startup, (setup_graphics))
    .add_systems(
        Update,
        setup_physics.run_if(input_just_pressed(KeyCode::KeyS)),
    );

    /*
    let check_positions_id = app.world.register_system(check_positions);
    let setup_physics_id = app.world.register_system(setup_physics);
    while app.plugins_state() == PluginsState::Adding {
        #[cfg(not(target_arch = "wasm32"))]
        bevy_tasks::tick_global_task_pools_on_main_thread();
    }
    app.finish();
    app.cleanup();
    app.update();
    app.world.run_system(setup_physics_id).expect("error1");
    app.world.run_system(check_positions_id).expect("error1");

    app.update();
    app.world.run_system(check_positions_id).expect("error2");
    app.update();
    app.world.run_system(check_positions_id).expect("error2");*/
    app.run();
}

pub fn setup_graphics(mut commands: Commands, mut rapier_context: ResMut<RapierContext>) {
    rapier_context.integration_parameters.num_solver_iterations = NonZeroUsize::new(1).unwrap();
    commands.spawn(Camera2dBundle {
        transform: Transform::from_xyz(0.0, -200.0, 0.0),
        ..default()
    });
}

#[derive(Component)]
struct DynamicBodyMarker;

pub fn setup_physics(mut commands: Commands) {
    let rad = 4.0;
    let shift = 10.0;
    let parent_entity = commands
        .spawn((
            TransformBundle::from(Transform::from_xyz(-shift, 0.0, 0.0)),
            RigidBody::Fixed,
            Collider::cuboid(rad, rad),
        ))
        .id();
    let child_entity = commands
        .spawn((
            TransformBundle::from(Transform::from_xyz(0.0, 0.0, 0.0)),
            RigidBody::Dynamic,
            Collider::cuboid(rad, rad),
            DynamicBodyMarker,
        ))
        .id();
    let joint = RevoluteJointBuilder::new().local_anchor2(Vec2::new(shift, 0.0));
    commands.entity(child_entity).with_children(|cmd| {
        // NOTE: we want to attach multiple impulse joints to this entity, so
        //       we need to add the components to children of the entity. Otherwise
        //       the second joint component would just overwrite the first one.
        cmd.spawn(ImpulseJoint::new(parent_entity, joint));
    });
}

fn check_positions(q: Query<(&Transform, &GlobalTransform), With<DynamicBodyMarker>>) {
    for (t, gt) in q.iter() {
        let gt = gt.compute_transform();
        dbg!(&gt);
        dbg!(&t);
    }
}
