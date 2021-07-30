extern crate rapier3d as rapier; // For the debug UI.

use bevy::{
    gltf::{Gltf, GltfMesh},
    prelude::*,
};
use bevy_rapier3d::{na::Point3, prelude::*};
use rapier3d::pipeline::PhysicsPipeline;
use std::convert::TryInto;
use ui::DebugUiPlugin;

#[path = "../../src_debug_ui/mod.rs"]
mod ui;

#[derive(Debug, Clone, Eq, PartialEq, Hash)]
enum AppState {
    GltfAssetsLoading,
    InGame,
}
#[derive(Default)]
struct AssetsLoading(Vec<HandleUntyped>);

fn setup_assets(server: Res<AssetServer>, mut loading: ResMut<AssetsLoading>) {
    println!("loading assets");
    // we can have different asset types
    let handle: Handle<Gltf> =
        server.load(format!("{}/../assets/Suzanne.gltf", env!("CARGO_MANIFEST_DIR")).as_str());
    // add them all to our collection for tracking
    loading.0.push(handle.clone_untyped());
}

fn check_assets_ready(
    mut commands: Commands,
    server: Res<AssetServer>,
    loading: Res<AssetsLoading>,
    mut app_state: ResMut<State<AppState>>,
) {
    use bevy::asset::LoadState;

    match server.get_group_load_state(loading.0.iter().map(|h| h.id)) {
        LoadState::Failed => {
            println!("Loading failed...");
            // one of our assets had an error
        }
        LoadState::Loaded => {
            println!("loading successful!");
            // all assets are now ready

            // this might be a good place to transition into your in-game state

            // remove the resource to drop the tracking handles
            app_state.set(AppState::InGame).unwrap();
            commands.remove_resource::<AssetsLoading>();

            // (note: if you don't have any other handles to the assets
            // elsewhere, they will get unloaded after this)
        }
        _ => {
            // NotLoaded/Loading: not fully ready yet
            println!("Loading...");
        }
    }
}

fn setup_physics(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    gltfs: Res<Assets<Gltf>>,
    gltf_meshes: Res<Assets<GltfMesh>>,
    meshes: Res<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let color = 0;

    /* Create the ground. */
    let collider = ColliderBundle {
        shape: ColliderShape::cuboid(5.0, 0.1, 5.0),
        ..Default::default()
    };

    commands
        .spawn_bundle(collider)
        .insert(ColliderDebugRender::with_id(color))
        .insert(ColliderPositionSync::Discrete);

    /* Create the bouncing ball. */
    let rigid_body = RigidBodyBundle {
        position: Vec3::new(0.0, 10.0, 0.0).into(),
        ..Default::default()
    };
    let collider = ColliderBundle {
        shape: ColliderShape::ball(0.5),
        material: ColliderMaterial {
            restitution: 0.7,
            ..Default::default()
        },
        ..Default::default()
    };

    commands
        .spawn_bundle(rigid_body)
        .insert_bundle(collider)
        // .insert(mesh)
        .insert(ColliderDebugRender::with_id(color))
        .insert(ColliderPositionSync::Discrete);

    // Then any asset in the folder can be accessed like this:
    let suzanne_gltf_handle: Handle<Gltf> = asset_server
        .load(format!("{}/../assets/Suzanne.gltf", env!("CARGO_MANIFEST_DIR")).as_str());
    let suzanne_gltf = gltfs.get(suzanne_gltf_handle).unwrap();
    let suzanne_gltf_mesh = suzanne_gltf.named_meshes.get("Suzanne").unwrap();

    let suzanne_primitive_mesh = gltf_meshes.get(suzanne_gltf_mesh).unwrap();
    let suzanne_handle = &suzanne_primitive_mesh.primitives[0].mesh;
    let suzanne_mesh = meshes.get(suzanne_handle.clone()).unwrap();

    let sharedshapetuple: (Vec<Point3<f32>>, Vec<[u32; 3]>) =
        SharedShapeMesh(suzanne_mesh.clone()).try_into().unwrap();

    let convx_coll: SharedShape = ColliderShape::convex_decomposition(
        sharedshapetuple.0.as_slice(),
        sharedshapetuple.1.as_slice(),
    );

    let rb = RigidBodyBundle {
        position: Vec3::new(0.0, 5.0, 0.0).into(),
        ..Default::default()
    };
    let coll = ColliderBundle {
        shape: convx_coll,
        material: ColliderMaterial {
            restitution: 0.7,
            ..Default::default()
        },
        ..Default::default()
    };
    commands
        .spawn_bundle(coll)
        .insert_bundle(rb)
        .insert_bundle(PbrBundle {
            mesh: suzanne_handle.clone(),
            material: materials.add(Color::rgb(0.5, 0.4, 0.3).into()),
            ..Default::default()
        })
        .insert(ColliderPositionSync::Discrete);
}

fn main() {
    App::build()
        .insert_resource(ClearColor(Color::rgb(
            0xF9 as f32 / 255.0,
            0xF9 as f32 / 255.0,
            0xFF as f32 / 255.0,
        )))
        .insert_resource(Msaa::default())
        .add_plugins(DefaultPlugins)
        .add_plugin(bevy_winit::WinitPlugin::default())
        // .add_plugin(bevy_wgpu::WgpuPlugin::default())
        .add_plugin(RapierPhysicsPlugin::<NoUserData>::default())
        .add_plugin(RapierRenderPlugin)
        .add_plugin(DebugUiPlugin)
        .add_startup_system(setup_graphics.system())
        .add_startup_system(enable_physics_profiling.system())
        .add_state(AppState::GltfAssetsLoading)
        .init_resource::<AssetsLoading>()
        .add_system_set(
            SystemSet::on_enter(AppState::GltfAssetsLoading).with_system(setup_assets.system()),
        )
        .add_system_set(
            SystemSet::on_update(AppState::GltfAssetsLoading)
                .with_system(check_assets_ready.system()),
        )
        .add_system_set(SystemSet::on_enter(AppState::InGame).with_system(setup_physics.system()))
        .run();
}

fn setup_graphics(mut commands: Commands) {
    commands.spawn_bundle(LightBundle {
        transform: Transform::from_translation(Vec3::new(100.0, 10.0, 200.0)),
        light: Light {
            intensity: 100_000.0,
            range: 3000.0,
            ..Default::default()
        },
        ..Default::default()
    });
    // camera
    commands.spawn_bundle(PerspectiveCameraBundle {
        transform: Transform::from_xyz(0., 10., 15.0).looking_at(Vec3::new(0.0, 5.0, 0.0), Vec3::Y),
        ..Default::default()
    });
}

fn enable_physics_profiling(mut pipeline: ResMut<PhysicsPipeline>) {
    pipeline.counters.enable()
}
