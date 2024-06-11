use crate::dynamics::MassProperties;
use crate::dynamics::ReadMassProperties;
use crate::plugin::RapierConfiguration;
use crate::plugin::RapierContext;
use crate::prelude::MassModifiedEvent;
use bevy::prelude::*;

/// System responsible for writing updated mass properties back into the [`ReadMassProperties`] component.
pub fn writeback_mass_properties(
    context: Res<RapierContext>,
    config: Res<RapierConfiguration>,

    mut mass_props: Query<&mut ReadMassProperties>,
    mut mass_modified: EventReader<MassModifiedEvent>,
) {
    if !config.physics_pipeline_active {
        return;
    }

    for (_, world) in context.worlds.iter() {
        for entity in mass_modified.read() {
            let Some(handle) = world.entity2body.get(entity).copied() else {
                continue;
            };

            let Some(rb) = world.bodies.get(handle) else {
                continue;
            };

            let Ok(mut mass_props) = mass_props.get_mut(**entity) else {
                continue;
            };

            let new_mass_props = MassProperties::from_rapier(rb.mass_properties().local_mprops);

            // NOTE: we write the new value only if there was an
            //       actual change, in order to not trigger bevy’s
            //       change tracking when the values didn’t change.
            if mass_props.get() != &new_mass_props {
                mass_props.set(new_mass_props);
            }
        }
    }
}
