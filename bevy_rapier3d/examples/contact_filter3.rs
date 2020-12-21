extern crate rapier3d as rapier; // For the debug UI.

use bevy::prelude::*;
use bevy::render::pass::ClearColor;
use bevy_rapier3d::physics::{InteractionPairFilters, RapierPhysicsPlugin};
use bevy_rapier3d::render::RapierRenderPlugin;
use rapier::geometry::{ContactPairFilter, PairFilterContext, SolverFlags};
use rapier3d::dynamics::RigidBodyBuilder;
use rapier3d::geometry::ColliderBuilder;
use rapier3d::pipeline::PhysicsPipeline;
use ui::DebugUiPlugin;

#[path = "../../src_debug_ui/mod.rs"]
mod ui;

// A custom filter that allows contacts only between rigid-bodies with the
// same user_data value.
// Note that using collision groups would be a more efficient way of doing
// this, but we use custom filters instead for demonstration purpose.
struct SameUserDataFilter;
impl ContactPairFilter for SameUserDataFilter {
    fn filter_contact_pair(&self, context: &PairFilterContext) -> Option<SolverFlags> {
        if context.rigid_body1.user_data == context.rigid_body2.user_data {
            Some(SolverFlags::COMPUTE_IMPULSES)
        } else {
            None
        }
    }
}

fn main() {
    App::build()
        .add_resource(ClearColor(Color::rgb(
            0xF9 as f32 / 255.0,
            0xF9 as f32 / 255.0,
            0xFF as f32 / 255.0,
        )))
        .add_resource(Msaa::default())
        .add_plugins(DefaultPlugins)
        .add_plugin(bevy_winit::WinitPlugin::default())
        .add_plugin(bevy_wgpu::WgpuPlugin::default())
        .add_plugin(RapierPhysicsPlugin)
        .add_plugin(RapierRenderPlugin)
        .add_plugin(DebugUiPlugin)
        .add_startup_system(setup_graphics.system())
        .add_startup_system(setup_physics.system())
        .add_startup_system(enable_physics_profiling.system())
        .run();
}

fn enable_physics_profiling(mut pipeline: ResMut<PhysicsPipeline>) {
    pipeline.counters.enable()
}

fn setup_graphics(commands: &mut Commands) {
    commands
        .spawn(LightBundle {
            transform: Transform::from_translation(Vec3::new(1000.0, 100.0, 2000.0)),
            ..Default::default()
        })
        .spawn(Camera3dBundle {
            transform: Transform::from_matrix(Mat4::face_toward(
                Vec3::new(-30.0, 30.0, 100.0),
                Vec3::new(0.0, 10.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
            )),
            ..Default::default()
        });
}

pub fn setup_physics(commands: &mut Commands) {
    /*
     * Ground
     */
    commands.insert_resource(InteractionPairFilters::new().contact_filter(SameUserDataFilter));

    let ground_size = 10.0;

    let rigid_body = RigidBodyBuilder::new_static()
        .translation(0.0, -10.0, 0.0)
        .user_data(0);
    let collider = ColliderBuilder::cuboid(ground_size, 1.2, ground_size);
    commands.spawn((rigid_body, collider));

    let rigid_body = RigidBodyBuilder::new_static().user_data(1);
    let collider = ColliderBuilder::cuboid(ground_size, 1.2, ground_size);
    commands.spawn((rigid_body, collider));

    /*
     * Create the cubes
     */
    let num = 4;
    let rad = 0.5;

    let shift = rad * 2.0;
    let centerx = shift * (num as f32) / 2.0;
    let centery = shift / 2.0;

    for i in 0..num {
        for j in 0usize..num * 5 {
            let x = (i as f32 + j as f32 * 0.2) * shift - centerx;
            let y = j as f32 * shift + centery + 2.0;

            // Build the rigid body.
            let body = RigidBodyBuilder::new_dynamic()
                .user_data(j as u128 % 2)
                .translation(x, y, 0.0);
            let collider = ColliderBuilder::cuboid(rad, rad, rad).density(1.0);
            commands.spawn((body, collider));
        }
    }
}
