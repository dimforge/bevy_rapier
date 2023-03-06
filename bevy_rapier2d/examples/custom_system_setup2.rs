use bevy::{core::FrameCount, prelude::*};
use bevy_rapier2d::prelude::*;

#[derive(Resource)]
struct SpecialSchedule {
    schedule: Schedule,
}

fn main() {
    let mut app = App::new();

    app.insert_resource(ClearColor(Color::rgb(
        0xF9 as f32 / 255.0,
        0xF9 as f32 / 255.0,
        0xFF as f32 / 255.0,
    )))
    .add_plugins(DefaultPlugins)
    .add_plugin(RapierDebugRenderPlugin::default())
    .add_startup_system(setup_graphics)
    .add_startup_system(setup_physics)
    .add_system(run_schedule.in_base_set(CoreSet::PostUpdate));

    // Do the setup however we want, maybe in its very own schedule
    let mut schedule = Schedule::new();

    schedule.configure_sets(
        (
            PhysicsSet::SyncBackend,
            PhysicsSet::SyncBackendFlush,
            PhysicsSet::StepSimulation,
            PhysicsSet::Writeback,
        )
            .chain(),
    );

    schedule.add_systems(
        RapierPhysicsPlugin::<NoUserData>::get_systems(PhysicsSet::SyncBackend)
            .in_base_set(PhysicsSet::SyncBackend),
    );

    // schedule.add_systems(
    //     RapierPhysicsPlugin::<NoUserData>::get_systems(PhysicsSet::SyncBackendFlush)
    //         .in_base_set(PhysicsSet::SyncBackendFlush),
    // );
    app.add_system(apply_system_buffers.in_base_set(PhysicsSet::SyncBackendFlush));

    schedule.add_systems(
        RapierPhysicsPlugin::<NoUserData>::get_systems(PhysicsSet::StepSimulation)
            .in_base_set(PhysicsSet::StepSimulation),
    );
    schedule.add_system(despawn_one_box.in_base_set(PhysicsSet::StepSimulation));

    schedule.add_systems(
        RapierPhysicsPlugin::<NoUserData>::get_systems(PhysicsSet::Writeback)
            .in_base_set(PhysicsSet::Writeback),
    );

    app.insert_resource(SpecialSchedule { schedule })
        .add_plugin(
            RapierPhysicsPlugin::<NoUserData>::default()
                .with_physics_scale(100.)
                .with_default_system_setup(false),
        )
        .run();
}

fn run_schedule(world: &mut World) {
    world.resource_scope(|world, mut special: Mut<SpecialSchedule>| {
        special.schedule.run(world);
    })
}

fn despawn_one_box(
    mut commands: Commands,
    frame_count: ResMut<FrameCount>,
    query: Query<Entity, (With<Collider>, With<RigidBody>)>,
) {
    // Delete a box every 10 frames
    if frame_count.0 % 10 == 0 && !query.is_empty() {
        let len = query.iter().len();
        // Get a "random" box to make sim interesting
        if let Some(entity) = query.iter().nth(frame_count.0 as usize % len) {
            commands.entity(entity).despawn();
        }
    }
}

fn setup_graphics(mut commands: Commands) {
    commands.spawn(Camera2dBundle {
        transform: Transform::from_xyz(0.0, 20.0, 0.0),
        ..default()
    });
}

pub fn setup_physics(mut commands: Commands) {
    /*
     * Ground
     */
    let ground_size = 500.0;
    let ground_height = 10.0;

    commands.spawn((
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

            commands.spawn((
                TransformBundle::from(Transform::from_xyz(x, y, 0.0)),
                RigidBody::Dynamic,
                Collider::cuboid(rad, rad),
            ));
        }

        offset -= 0.05 * rad * (num as f32 - 1.0);
    }
}
