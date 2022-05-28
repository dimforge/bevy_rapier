use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

struct SpecialStagingPlugin {
    schedule: Schedule,
}

impl SpecialStagingPlugin {
    pub fn new(schedule: Schedule) -> Self {
        Self { schedule }
    }
}

impl SpecialStagingPlugin {
    fn build(self, app: &mut App) {
        app.add_stage_after(
            CoreStage::Update,
            "special_staging_plugin_stage",
            SpecialStage::new(self.schedule),
        );
    }
}

struct SpecialStage {
    schedule: Schedule,
}

impl SpecialStage {
    pub fn new(schedule: Schedule) -> Self {
        Self { schedule }
    }
}

impl Stage for SpecialStage {
    fn run(&mut self, world: &mut World) {
        self.schedule.run_once(world);
    }
}

struct FrameCount(u32);

fn main() {
    let mut app = App::new();

    app.insert_resource(ClearColor(Color::rgb(
        0xF9 as f32 / 255.0,
        0xF9 as f32 / 255.0,
        0xFF as f32 / 255.0,
    )))
    .insert_resource(Msaa::default())
    .insert_resource(FrameCount(0))
    .add_plugins(DefaultPlugins)
    .add_plugin(RapierDebugRenderPlugin::default())
    .add_startup_system(setup_graphics)
    .add_startup_system(setup_physics);

    // Do the stage setup however we want, maybe in a special plugin that has
    // its very own schedule
    SpecialStagingPlugin::new(
        Schedule::default()
            .with_stage(
                PhysicsStages::SyncBackend,
                SystemStage::parallel().with_system_set(
                    RapierPhysicsPlugin::<NoUserData>::get_systems(PhysicsStages::SyncBackend),
                ),
            )
            .with_stage_after(
                PhysicsStages::SyncBackend,
                PhysicsStages::StepSimulation,
                SystemStage::parallel()
                    .with_system(despawn_one_box) // We can add a special despawn to determine cleanup later
                    .with_system_set(RapierPhysicsPlugin::<NoUserData>::get_systems(
                        PhysicsStages::StepSimulation,
                    )),
            )
            .with_stage_after(
                PhysicsStages::StepSimulation,
                PhysicsStages::Writeback,
                SystemStage::parallel().with_system_set(
                    RapierPhysicsPlugin::<NoUserData>::get_systems(PhysicsStages::Writeback),
                ),
            ),
    )
    .build(&mut app);

    // Be sure to setup all four stages
    app.add_stage_before(
        CoreStage::Last,
        PhysicsStages::DetectDespawn,
        SystemStage::parallel().with_system_set(RapierPhysicsPlugin::<NoUserData>::get_systems(
            PhysicsStages::DetectDespawn,
        )),
    );

    app.add_plugin(RapierPhysicsPlugin::<NoUserData>::default().with_default_system_setup(false));

    app.run();
}

fn despawn_one_box(
    mut commands: Commands,
    mut frame_count: ResMut<FrameCount>,
    query: Query<Entity, (With<Collider>, With<RigidBody>)>,
) {
    frame_count.0 += 1;

    // Delete a box every 10 frames
    if frame_count.0 % 10 == 0 && !query.is_empty() {
        let count = query.iter().count();
        if let Some(entity) = query
            .iter()
            .skip(frame_count.0 as usize % count) // Get a "random" box to make sim interesting
            .take(1)
            .next()
        {
            commands.entity(entity).despawn();
        }
    }
}

fn setup_graphics(mut commands: Commands) {
    commands.spawn_bundle(PerspectiveCameraBundle {
        transform: Transform::from_matrix(
            Mat4::look_at_rh(
                Vec3::new(-30.0, 30.0, 100.0),
                Vec3::new(0.0, 10.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
            )
            .inverse(),
        ),
        ..Default::default()
    });
}

pub fn setup_physics(mut commands: Commands) {
    /*
     * Ground
     */
    let ground_size = 200.1;
    let ground_height = 0.1;

    commands
        .spawn_bundle(TransformBundle::from(Transform::from_xyz(0.0, -ground_height, 0.0))
        .insert(Collider::cuboid(ground_size, ground_height, ground_size));

    /*
     * Create the cubes
     */
    let num = 8;
    let rad = 1.0;

    let shift = rad * 2.0 + rad;
    let centerx = shift * (num / 2) as f32;
    let centery = shift / 2.0;
    let centerz = shift * (num / 2) as f32;

    let mut offset = -(num as f32) * (rad * 2.0 + rad) * 0.5;
    let mut color = 0;
    let colors = [
        Color::hsl(220.0, 1.0, 0.3),
        Color::hsl(180.0, 1.0, 0.3),
        Color::hsl(260.0, 1.0, 0.7),
    ];

    for j in 0usize..20 {
        for i in 0..num {
            for k in 0usize..num {
                let x = i as f32 * shift - centerx + offset;
                let y = j as f32 * shift + centery + 3.0;
                let z = k as f32 * shift - centerz + offset;
                color += 1;

                commands
                    .spawn_bundle(TransformBundle::from(Transform::from_xyz(x, y, z))
                    .insert(RigidBody::Dynamic)
                    .insert(Collider::cuboid(rad, rad, rad))
                    .insert(ColliderDebugColor(colors[color % 3]));
            }
        }

        offset -= 0.05 * rad * (num as f32 - 1.0);
    }
}
