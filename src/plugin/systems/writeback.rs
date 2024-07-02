use crate::dynamics::MassProperties;
use crate::dynamics::ReadMassProperties;
use crate::plugin::context::RapierContextEntityLink;
use crate::plugin::RapierConfiguration;
use crate::plugin::RapierContext;
use crate::prelude::MassModifiedEvent;
use bevy::prelude::*;

/// System responsible for writing updated mass properties back into the [`ReadMassProperties`] component.
pub fn writeback_mass_properties(
    link: Query<&RapierContextEntityLink>,
    context: Query<&RapierContext>,
    config: Query<&RapierConfiguration>,

    mut mass_props: Query<&mut ReadMassProperties>,
    mut mass_modified: EventReader<MassModifiedEvent>,
) {
    for entity in mass_modified.read() {
        let link = link.get(entity.0).unwrap();
        let config = config.get(link.0).unwrap();
        if config.physics_pipeline_active {
            let context = context.get(link.0).unwrap();

            if let Some(handle) = context.entity2body.get(entity).copied() {
                if let Some(rb) = context.bodies.get(handle) {
                    if let Ok(mut mass_props) = mass_props.get_mut(**entity) {
                        let new_mass_props =
                            MassProperties::from_rapier(rb.mass_properties().local_mprops);

                        // NOTE: we write the new value only if there was an
                        //       actual change, in order to not trigger bevy’s
                        //       change tracking when the values didn’t change.
                        if mass_props.get() != &new_mass_props {
                            mass_props.set(new_mass_props);
                        }
                    }
                }
            }
        }
    }
}
