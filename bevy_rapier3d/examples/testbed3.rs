mod boxes3;
mod despawn3;
mod events3;
mod joints3;
mod joints_despawn3;
mod locked_rotations3;
mod multiple_colliders3;
mod ray_casting3;
mod static_trimesh3;

use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

#[derive(Debug, Clone, Eq, PartialEq, Default, Hash, States)]
pub enum Examples {
    #[default]
    None,
    Boxes3,
    Despawn3,
    Events3,
    Joints3,
    JointsDespawn3,
    LockedRotations3,
    MultipleColliders3,
    Raycasting3,
    StaticTrimesh3,
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
            RapierPhysicsPlugin::<NoUserData>::default(),
            RapierDebugRenderPlugin::default(),
        ))
        //
        //boxes2
        .add_systems(
            OnEnter(Examples::Boxes3),
            (boxes3::setup_graphics, boxes3::setup_physics),
        )
        .add_systems(OnExit(Examples::Boxes3), cleanup)
        //
        // despawn
        .init_resource::<despawn3::DespawnResource>()
        .add_systems(
            OnEnter(Examples::Despawn3),
            (despawn3::setup_graphics, despawn3::setup_physics),
        )
        .add_systems(
            Update,
            despawn3::despawn.run_if(in_state(Examples::Despawn3)),
        )
        .add_systems(OnExit(Examples::Despawn3), cleanup)
        //
        // events
        .add_systems(
            OnEnter(Examples::Events3),
            (events3::setup_graphics, events3::setup_physics),
        )
        .add_systems(
            Update,
            events3::display_events.run_if(in_state(Examples::Events3)),
        )
        .add_systems(OnExit(Examples::Events3), cleanup)
        //
        // joints
        .add_systems(
            OnEnter(Examples::Joints3),
            (joints3::setup_graphics, joints3::setup_physics),
        )
        .add_systems(OnExit(Examples::Joints3), cleanup)
        //
        // joints despawn
        .init_resource::<joints_despawn3::DespawnResource>()
        .add_systems(
            OnEnter(Examples::JointsDespawn3),
            (
                joints_despawn3::setup_graphics,
                joints_despawn3::setup_physics,
            ),
        )
        .add_systems(
            Update,
            joints_despawn3::despawn.run_if(in_state(Examples::JointsDespawn3)),
        )
        .add_systems(OnExit(Examples::JointsDespawn3), cleanup)
        //
        // locked rotations
        .add_systems(
            OnEnter(Examples::LockedRotations3),
            (
                locked_rotations3::setup_graphics,
                locked_rotations3::setup_physics,
            ),
        )
        .add_systems(OnExit(Examples::LockedRotations3), cleanup)
        //
        // multiple colliders
        .add_systems(
            OnEnter(Examples::MultipleColliders3),
            (
                multiple_colliders3::setup_graphics,
                multiple_colliders3::setup_physics,
            ),
        )
        .add_systems(OnExit(Examples::MultipleColliders3), cleanup)
        //
        // raycasting
        .add_systems(
            OnEnter(Examples::Raycasting3),
            (ray_casting3::setup_graphics, ray_casting3::setup_physics),
        )
        .add_systems(
            Update,
            ray_casting3::cast_ray.run_if(in_state(Examples::Raycasting3)),
        )
        .add_systems(OnExit(Examples::Raycasting3), cleanup)
        //
        // static trimesh
        .init_resource::<static_trimesh3::BallState>()
        .add_systems(
            OnEnter(Examples::StaticTrimesh3),
            (
                static_trimesh3::setup_graphics,
                static_trimesh3::setup_physics,
            ),
        )
        .add_systems(
            Update,
            static_trimesh3::ball_spawner.run_if(in_state(Examples::StaticTrimesh3)),
        )
        .add_systems(OnExit(Examples::StaticTrimesh3), cleanup)
        //
        //testbed
        .add_systems(
            OnEnter(Examples::None),
            |mut next_state: ResMut<NextState<Examples>>| {
                next_state.set(Examples::Boxes3);
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
            Examples::None => Examples::Boxes3,
            Examples::Boxes3 => Examples::Despawn3,
            Examples::Despawn3 => Examples::Events3,
            Examples::Events3 => Examples::Joints3,
            Examples::Joints3 => Examples::JointsDespawn3,
            Examples::JointsDespawn3 => Examples::LockedRotations3,
            Examples::LockedRotations3 => Examples::MultipleColliders3,
            Examples::MultipleColliders3 => Examples::Raycasting3,
            Examples::Raycasting3 => Examples::StaticTrimesh3,
            Examples::StaticTrimesh3 => Examples::Boxes3,
        };
        next_state.set(next);
    }
}
