mod boxes2;
mod debug_despawn2;
mod despawn2;
mod events2;
mod joints2;
mod joints_despawn2;
mod locked_rotations2;
mod multiple_colliders2;
mod player_movement2;
mod rope_joint2;

use bevy::prelude::*;
use bevy_rapier2d::prelude::*;

#[derive(Debug, Clone, Eq, PartialEq, Default, Hash, States)]
pub enum Examples {
    #[default]
    None,
    Boxes2,
    RopeJoint2,
    DebugDespawn2,
    Despawn2,
    Events2,
    Joints2,
    JointsDespawn2,
    LockedRotation2,
    MultipleColliders2,
    PlayerMovement2,
}

#[derive(Resource, Default)]
struct ExamplesRes {
    entities_before: Vec<Entity>,
}

fn main() {
    let mut app = App::new();
    app.add_state::<Examples>()
        .init_resource::<ExamplesRes>()
        .add_plugins((
            DefaultPlugins,
            RapierPhysicsPlugin::<NoUserData>::pixels_per_meter(100.0),
            RapierDebugRenderPlugin::default(),
        ))
        //
        //boxes2
        .add_systems(
            OnEnter(Examples::Boxes2),
            (boxes2::setup_graphics, boxes2::setup_physics),
        )
        .add_systems(OnExit(Examples::Boxes2), cleanup)
        //
        // rope joint
        .add_systems(
            OnEnter(Examples::RopeJoint2),
            (rope_joint2::setup_graphics, rope_joint2::setup_physics),
        )
        .add_systems(OnExit(Examples::RopeJoint2), cleanup)
        //
        //debug despawn
        .init_resource::<debug_despawn2::Game>()
        .add_systems(OnEnter(Examples::DebugDespawn2), debug_despawn2::setup_game)
        .add_systems(
            Update,
            debug_despawn2::cube_sleep_detection.run_if(in_state(Examples::DebugDespawn2)),
        )
        .add_systems(OnExit(Examples::DebugDespawn2), cleanup)
        //
        //despawn2
        .insert_resource(despawn2::DespawnResource::default())
        .insert_resource(despawn2::ResizeResource::default())
        .add_systems(
            OnEnter(Examples::Despawn2),
            (despawn2::setup_graphics, despawn2::setup_physics),
        )
        .add_systems(
            Update,
            (despawn2::despawn, despawn2::resize).run_if(in_state(Examples::Despawn2)),
        )
        .add_systems(OnExit(Examples::Despawn2), cleanup)
        //
        //events
        .add_systems(
            OnEnter(Examples::Events2),
            (events2::setup_graphics, events2::setup_physics),
        )
        .add_systems(
            Update,
            events2::display_events.run_if(in_state(Examples::Events2)),
        )
        .add_systems(OnExit(Examples::Events2), cleanup)
        //
        //events
        .add_systems(
            OnEnter(Examples::Joints2),
            (joints2::setup_graphics, joints2::setup_physics),
        )
        .add_systems(OnExit(Examples::Joints2), cleanup)
        //
        //despawn2
        .insert_resource(joints_despawn2::DespawnResource::default())
        .add_systems(
            OnEnter(Examples::JointsDespawn2),
            (
                joints_despawn2::setup_graphics,
                joints_despawn2::setup_physics,
            ),
        )
        .add_systems(
            Update,
            (joints_despawn2::despawn).run_if(in_state(Examples::JointsDespawn2)),
        )
        .add_systems(OnExit(Examples::JointsDespawn2), cleanup)
        //
        //locked rotations
        .add_systems(
            OnEnter(Examples::LockedRotation2),
            (
                locked_rotations2::setup_graphics,
                locked_rotations2::setup_physics,
            ),
        )
        .add_systems(OnExit(Examples::LockedRotation2), cleanup)
        //
        //multiple colliders
        .add_systems(
            OnEnter(Examples::MultipleColliders2),
            (
                multiple_colliders2::setup_graphics,
                multiple_colliders2::setup_physics,
            ),
        )
        .add_systems(OnExit(Examples::MultipleColliders2), cleanup)
        //
        //player movement
        .add_systems(
            OnEnter(Examples::PlayerMovement2),
            player_movement2::spawn_player,
        )
        .add_systems(
            Update,
            (player_movement2::player_movement).run_if(in_state(Examples::PlayerMovement2)),
        )
        .add_systems(OnExit(Examples::PlayerMovement2), cleanup)
        //
        //testbed
        .add_systems(
            OnEnter(Examples::None),
            |mut next_state: ResMut<NextState<Examples>>| {
                next_state.set(Examples::Boxes2);
            },
        )
        .add_systems(OnExit(Examples::None), init)
        .add_systems(Update, check_toggle);

    app.run();
}

fn init(world: &mut World) {
    //save all entities that are in the world before setting up any example
    // to be able to always return to this state when switching from one example to the other
    world.resource_mut::<ExamplesRes>().entities_before =
        world.iter_entities().map(|e| e.id()).collect::<Vec<_>>();
}

fn cleanup(world: &mut World) {
    let keep_alive = world.resource::<ExamplesRes>().entities_before.clone();

    let remove = world
        .iter_entities()
        .filter_map(|e| (!keep_alive.contains(&e.id())).then_some(e.id()))
        .collect::<Vec<_>>();

    for r in remove {
        world.despawn(r);
    }
}

fn check_toggle(
    state: Res<State<Examples>>,
    mut next_state: ResMut<NextState<Examples>>,
    mouse_input: Res<Input<MouseButton>>,
) {
    if mouse_input.just_pressed(MouseButton::Left) {
        let next = match *state.get() {
            Examples::None => Examples::Boxes2,
            Examples::Boxes2 => Examples::RopeJoint2,
            Examples::RopeJoint2 => Examples::DebugDespawn2,
            Examples::DebugDespawn2 => Examples::Despawn2,
            Examples::Despawn2 => Examples::Events2,
            Examples::Events2 => Examples::Joints2,
            Examples::Joints2 => Examples::JointsDespawn2,
            Examples::JointsDespawn2 => Examples::LockedRotation2,
            Examples::LockedRotation2 => Examples::MultipleColliders2,
            Examples::MultipleColliders2 => Examples::PlayerMovement2,
            Examples::PlayerMovement2 => Examples::Boxes2,
        };
        next_state.set(next);
    }
}
