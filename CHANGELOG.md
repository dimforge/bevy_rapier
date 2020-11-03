# Changelog

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

