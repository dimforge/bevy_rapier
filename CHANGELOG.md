# Changelog

## Unreleased
### Fixed
- Add the missing `init_async_scene_colliders` to the list of the plugin systems.

## 0.14.0 (31 May 2022)
### Added
- Add the `AsyncSceneCollider` component to generate collision for scene meshes similar to `AsyncCollider`.
- Add calls to `App::register_type` for the types implementing `Reflect`.

### Modified
- `Collider::bevy_mesh`, `Collider::bevy_mesh_convex_decomposition` and `Collider::bevy_mesh_convex_decomposition_with_params` was replaced with single `Collider::from_bevy_mesh` function which accepts `ComputedColliderShape`.
- `AsyncCollider` is now a struct which contains a mesh handle and `ComputedColliderShape`.
- The physics systems are now running after `CoreStage::Update` but before `CoreStage::PostUpdate`.
- The collider and rigid-body positions are read from the `GlobalTransform` instead of `Transform` (the 
  transforms modified by Rapier are written back to the `Transform` component). It is therefore important
  to insert both a `Transform` and `GlobalTransform` component (or the `TransformBundle` bundle).
- It is now possible to prevent the plugin from registering its system thanks to `RapierPhysicsPlugin::with_default_system_setup(false)`.
  If that’s the case, the `RapierPhysicsPlugin::get_systems` method can be called to retrieve the relevant `SystemSet`
  that can be added to your own stages in order to apply your own scheduling.

### Fixed
- Fixed issues where contact regularization (using compliance) would result in tunnelling despite tunnelling
  being enabled.
- Don’t overwrite the user’s `RapierConfiguration` if one already exists before initializing the plugin.

## 0.13.2 (5 May 2022)
### Modified
- The `TimestepMode` and `SimulationToRenderTime` structures are now public.

### Fixed
- Fixed colors rendered by the debug-renderer.
- Fix issue where the debug-renderer would sometimes render lines behind the user’s meshes/sprites.


## 0.13.1 (1 May 2022)
### Added
- Add the `CollidingEntities` components which tracks the set of entities colliding
  with a given entity.
- Add constructors `Velocity::linear`, `Velocity::angular`, `Ccd::enabled()`, `Ccd::disabled()`,
  `Dominance::group`, `Friction::coefficient`, `Restitution::coefficient`, `CollisionGroups::new`,
  `SolverGroups::new`.
- Add `RapierContext::collider_parent` that returns the entity containing the parent `RigidBody`
  of a collider.

### Modified
- Switched to linear ordering to order our systems.
- Make the `plugin::systems` module public.


## 0.13.0 (30 Apr. 2022)
This is a **complete rewrite of the plugin** (mostly likely the last large redesign this plugin
will be subject to). It switches to `rapier` 0.12 and `bevy` 0.7. The focus of this rewrite was
to significantly improve ergonomics while simplifying the codebase and adding new features.

Refer to [#138](https://github.com/dimforge/bevy_rapier/pull/138) for extensive details
on this change. See the folders `bevy_rapier2d/examples` and `bevy_rapier3d/examples`
for some examples.

## 0.12.0
### Modified
- Switch to `rapier` 0.12.0-alpha.0 and `nalgebra` 0.30.
- Switch to `bevy` 0.6.
- All the Rapier components have been wrapped into wrapper types (for example `ColliderPosition`
  has been wrapped into `ColliderPositionComponent`). These wrapper types are the ones that need
  to be used as bevy components. To convert a Rapier component to it’s corresponding Bevy component,
  simply use `.into()`. See the examples in `bevy_rapier2d/examples` and `bevy_rapier3d/examples`
  for details.

## 0.11.0
### Modified
- Switch to `Rapier` 0.11 and `nalgebra` 0.29.
- Add labels to each system from `bevy-rapier`.

### Fixed
- Fix panics when despawning joints or colliders.
- Fix a panic where adding a collider.
- Don’t let the plugin overwrite the user’s `PhysicsHooksWithQueryObject` if it was already 
  present before inserting the plugin.

## 0.10.2
### Fixed
- Fix build when targeting WASM.

## 0.10.1
### Fixed
- Fix joint removal when despawning its entity.

## 0.10.0
A new, exhaustive, user-guide for bevy_rapier has been uploaded to rapier.rs.

This version is a complete rewrite of the plugin. Rigid-bodies and
colliders are now split into components that can be queried like any other
components. They are created by inserting a `RigidBodyBundle` and/or a `ColliderBundle`.

In addition, the `Entity` type and `ColliderHandle/RigidBodyHandle` type can now be
converted directly using `entity.handle()` or `handle.entity()`.

Finally, there is now a prelude: `use bevy_rapier2d::prelude::*`.


## 0.9.0
### Added
- The `ColliderDebugRender` component must be added to an entity
  containing a collider shape in order to render it.
  
## 0.8.0
### Changed
- Use the version 0.5.0 of Rapier.

### Added
- The debug rendering of 3D trimesh now work.
- The debug rendering won't crash any more if a not-yet-supported shape
  is used. It will silently ignore the shape instead.
- The crate has now a `render` feature that allows building it without any
  rendering support (avoiding some dependencies that may not compile when
  targetting WASM).

## 0.7.0
### Changed
- Use the version 0.4.0 of Bevy.

## 0.6.2
### Changed
- Fix the rendering of colliders attached to an entity children of another
  entity containing the rigid-body it is attached to.

## 0.6.1
### Changed
- The adaptive change of number of timesteps executed during each render loop
  (introduced in the version 0.4.0 with position interpolation) is
  now disabled by default. It needs to be enabled explicitly by setting
  `RapierConfiguration.time_dependent_number_of_timesteps` to `true`.

## 0.6.0
### Added
- It is now possible to attach multiple colliders to a single
  rigid-body by using Bevy hierarchy: an entity contains
  the `RigidBodyBuider` whereas its children contain the `ColliderBuilder`. 

### Changed
- We now use the latest version of Rapier: 0.4.0. See the
  [Rapier changelog](https://github.com/dimforge/rapier/blob/master/CHANGELOG.md#v040)
  for details. In particular, this includes the ability to lock the rotations of a rigid-body.

## 0.5.0
### Changed
- We now use the latest version of Bevy: 0.3.0

### Added
- Rigid-bodies, colliders, and joints, will automatically removed from the Rapier
  sets when their corresponding Bevy components are removed.

## 0.4.0
This release update the plugin to the latest version of Rapier, which itself
includes lots of new features. Refer to the Rapier changelogs for details.

### Added
- A `InteractionPairFilters` resource where you can your own filters for contact pair
and proximity pair filtering. Before considering the use of a custom filter, consider
using collision groups instead (as it is faster, but less versatile).

### Changed
- The stepping system now applies interpolation between two physics state at render time.
  Refer to [that issue](https://github.com/dimforge/bevy_rapier/pull/19) and the following
  blog post article for details: https://www.gafferongames.com/post/fix_your_timestep/

## 0.3.1
### Changed
- Rapier configuration
  - Replaced `Gravity` and `RapierPhysicsScale` resources with a unique `RapierConfiguration` resource.
  - Added an `physics_pipeline_active` attribute to `RapierConfiguration` allowing to pause the physic simulation.
  - Added a `query_pipeline_active` attribute to `RapierConfiguration` allowing to pause the query pipeline update.

