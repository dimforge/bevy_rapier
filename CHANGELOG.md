# Changelog

## 0.3.1

### Changed
- Rapier configuration
  - Replaced `Gravity` and `RapierPhysicsScale` resources with a unique `RapierConfiguration` resource.
  - Added an `physics_pipeline_active` attribute to `RapierConfiguration` allowing to pause the physic simulation.
  - Added a `query_pipeline_active` attribute to `RapierConfiguration` allowing to pause the query pipeline update.

