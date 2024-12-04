use crate::math::{Real, Vect};
use crate::plugin::context::{RapierContextColliders, RapierContextSimulation, RapierRigidBodySet};
use bevy::prelude::*;
use rapier::geometry::{Contact, ContactManifold, ContactPair, SolverContact, SolverFlags};

impl RapierContextSimulation {
    /// All the contact pairs involving the non-sensor collider attached to the given entity.
    ///
    /// The returned contact pairs identify pairs of colliders with intersecting bounding-volumes.
    /// To check if any geometric contact happened between the collider shapes, check
    /// [`ContactPairView::has_any_active_contact`].
    pub fn contact_pairs_with<'a, 'b: 'a>(
        &'a self,
        context_colliders: &'b RapierContextColliders,
        rigidbody_set: &'b RapierRigidBodySet,
        collider: Entity,
    ) -> impl Iterator<Item = ContactPairView> {
        context_colliders
            .entity2collider
            .get(&collider)
            .into_iter()
            .flat_map(|h| {
                self.narrow_phase
                    .contact_pairs_with(*h)
                    .map(|raw| ContactPairView {
                        context_colliders,
                        rigidbody_set,
                        raw,
                    })
            })
    }

    /// All the intersection pairs involving the collider attached to the given entity, where at least one collider
    /// involved in the intersection is a sensor.    
    ///
    /// The returned contact pairs identify pairs of colliders (where at least one is a sensor) with
    /// intersecting bounding-volumes. To check if any geometric overlap happened between the collider shapes, check
    /// the returned boolean.
    pub fn intersection_pairs_with<'a, 'b: 'a>(
        &'a self,
        rapier_colliders: &'b RapierContextColliders,
        collider: Entity,
    ) -> impl Iterator<Item = (Entity, Entity, bool)> + '_ {
        rapier_colliders
            .entity2collider
            .get(&collider)
            .into_iter()
            .flat_map(|h| {
                self.narrow_phase
                    .intersection_pairs_with(*h)
                    .filter_map(|(h1, h2, inter)| {
                        let e1 = rapier_colliders.collider_entity(h1);
                        let e2 = rapier_colliders.collider_entity(h2);
                        match (e1, e2) {
                            (Some(e1), Some(e2)) => Some((e1, e2, inter)),
                            _ => None,
                        }
                    })
            })
    }

    /// The contact pair involving two specific colliders.
    ///
    /// If this returns `None`, there is no contact between the two colliders.
    /// If this returns `Some`, then there may be a contact between the two colliders. Check the
    /// result [`ContactPairView::has_any_active_contact`] method to see if there is an actual contact.
    pub fn contact_pair<'a, 'b: 'a>(
        &'a self,
        context_colliders: &'b RapierContextColliders,
        rigidbody_set: &'b RapierRigidBodySet,
        collider1: Entity,
        collider2: Entity,
    ) -> Option<ContactPairView> {
        let h1 = context_colliders.entity2collider.get(&collider1)?;
        let h2 = context_colliders.entity2collider.get(&collider2)?;
        self.narrow_phase
            .contact_pair(*h1, *h2)
            .map(|raw| ContactPairView {
                context_colliders,
                rigidbody_set,
                raw,
            })
    }

    /// The intersection pair involving two specific colliders (at least one being a sensor).
    ///
    /// If this returns `None` or `Some(false)`, then there is no intersection between the two colliders.
    /// If this returns `Some(true)`, then there may be an intersection between the two colliders.
    pub fn intersection_pair(
        &self,
        rapier_colliders: &RapierContextColliders,
        collider1: Entity,
        collider2: Entity,
    ) -> Option<bool> {
        let h1 = rapier_colliders.entity2collider.get(&collider1)?;
        let h2 = rapier_colliders.entity2collider.get(&collider2)?;
        self.narrow_phase.intersection_pair(*h1, *h2)
    }

    /// All the contact pairs detected during the last timestep.
    pub fn contact_pairs<'a, 'b: 'a>(
        &'a self,
        context_colliders: &'b RapierContextColliders,
        rigidbody_set: &'b RapierRigidBodySet,
    ) -> impl Iterator<Item = ContactPairView> {
        self.narrow_phase
            .contact_pairs()
            .map(|raw| ContactPairView {
                context_colliders,
                rigidbody_set,
                raw,
            })
    }

    /// All the intersection pairs detected during the last timestep.
    pub fn intersection_pairs<'a, 'b: 'a>(
        &'a self,
        rapier_colliders: &'b RapierContextColliders,
    ) -> impl Iterator<Item = (Entity, Entity, bool)> + '_ {
        self.narrow_phase
            .intersection_pairs()
            .filter_map(|(h1, h2, inter)| {
                let e1 = rapier_colliders.collider_entity(h1);
                let e2 = rapier_colliders.collider_entity(h2);
                match (e1, e2) {
                    (Some(e1), Some(e2)) => Some((e1, e2, inter)),
                    _ => None,
                }
            })
    }
}

/// Read-only access to the properties of a contact manifold.
pub struct ContactManifoldView<'a> {
    rigidbody_set: &'a RapierRigidBodySet,
    /// The raw contact manifold from Rapier.
    pub raw: &'a ContactManifold,
}

impl<'a> ContactManifoldView<'a> {
    /// The number of points on this contact manifold.
    pub fn num_points(&self) -> usize {
        self.raw.points.len()
    }

    /// Retrieves the i-th point of this contact manifold.
    pub fn point(&self, i: usize) -> Option<ContactView> {
        self.raw.points.get(i).map(|raw| ContactView { raw })
    }

    /// The contacts points.
    pub fn points(&self) -> impl ExactSizeIterator<Item = ContactView> {
        self.raw.points.iter().map(|raw| ContactView { raw })
    }

    /// The contact normal of all the contacts of this manifold, expressed in the local space of the first shape.
    pub fn local_n1(&self) -> Vect {
        self.raw.local_n1.into()
    }

    /// The contact normal of all the contacts of this manifold, expressed in the local space of the second shape.
    pub fn local_n2(&self) -> Vect {
        self.raw.local_n2.into()
    }

    /// The first subshape involved in this contact manifold.
    ///
    /// This is zero if the first shape is not a composite shape.
    pub fn subshape1(&self) -> u32 {
        self.raw.subshape1
    }

    /// The second subshape involved in this contact manifold.
    ///
    /// This is zero if the second shape is not a composite shape.
    pub fn subshape2(&self) -> u32 {
        self.raw.subshape2
    }

    /// The first rigid-body involved in this contact manifold.
    pub fn rigid_body1(&self) -> Option<Entity> {
        self.raw
            .data
            .rigid_body1
            .and_then(|h| self.rigidbody_set.rigid_body_entity(h))
    }

    /// The second rigid-body involved in this contact manifold.
    pub fn rigid_body2(&self) -> Option<Entity> {
        self.raw
            .data
            .rigid_body2
            .and_then(|h| self.rigidbody_set.rigid_body_entity(h))
    }

    /// Flags used to control some aspects of the constraints solver for this contact manifold.
    pub fn solver_flags(&self) -> SolverFlags {
        self.raw.data.solver_flags
    }

    /// The world-space contact normal shared by all the contact in this contact manifold.
    pub fn normal(&self) -> Vect {
        self.raw.data.normal.into()
    }

    /// The contacts that will be seen by the constraints solver for computing forces.
    pub fn num_solver_contacts(&self) -> usize {
        self.raw.data.solver_contacts.len()
    }

    /// Gets the i-th solver contact.
    pub fn solver_contact(&self, i: usize) -> Option<SolverContactView> {
        self.raw
            .data
            .solver_contacts
            .get(i)
            .map(|raw| SolverContactView { raw })
    }

    /// The contacts that will be seen by the constraints solver for computing forces.
    pub fn solver_contacts(&self) -> impl ExactSizeIterator<Item = SolverContactView> {
        self.raw
            .data
            .solver_contacts
            .iter()
            .map(|raw| SolverContactView { raw })
    }

    /// The relative dominance of the bodies involved in this contact manifold.
    pub fn relative_dominance(&self) -> i16 {
        self.raw.data.relative_dominance
    }

    /// A user-defined piece of data.
    pub fn user_data(&self) -> u32 {
        self.raw.data.user_data
    }
}

impl<'a> ContactManifoldView<'a> {
    /// Returns the contact with the smallest distance (i.e. the largest penetration depth).
    pub fn find_deepest_contact(&self) -> Option<ContactView> {
        self.raw
            .find_deepest_contact()
            .map(|raw| ContactView { raw })
    }
}

/// Read-only access to the properties of a single contact.
pub struct ContactView<'a> {
    /// The raw contact from Rapier.
    pub raw: &'a Contact,
}

impl<'a> ContactView<'a> {
    /// The contact point in the local-space of the first shape.
    pub fn local_p1(&self) -> Vect {
        self.raw.local_p1.into()
    }

    /// The contact point in the local-space of the second shape.
    pub fn local_p2(&self) -> Vect {
        self.raw.local_p2.into()
    }

    /// The distance between the two contact points.
    pub fn dist(&self) -> Real {
        self.raw.dist
    }

    /// The feature ID of the first shape involved in the contact.
    pub fn fid1(&self) -> u32 {
        self.raw.fid1.0
    }

    /// The feature ID of the second shape involved in the contact.
    pub fn fid2(&self) -> u32 {
        self.raw.fid2.0
    }

    /// The impulse, along the contact normal, applied by this contact to the first collider's rigid-body.
    ///
    /// The impulse applied to the second collider's rigid-body is given by `-impulse`.
    pub fn impulse(&self) -> Real {
        self.raw.data.impulse
    }

    /// The friction impulse along the vector orthonormal to the contact normal, applied to the first
    /// collider's rigid-body.
    #[cfg(feature = "dim2")]
    pub fn tangent_impulse(&self) -> Real {
        self.raw.data.tangent_impulse.x
    }

    /// The friction impulse along the vector orthonormal to the contact normal, applied to the first
    /// collider's rigid-body.
    #[cfg(feature = "dim3")]
    pub fn tangent_impulse(&self) -> [Real; 2] {
        self.raw.data.tangent_impulse.into()
    }
}

/// Read-only access to the properties of a single solver contact.
pub struct SolverContactView<'a> {
    /// The raw solver contact from Rapier.
    pub raw: &'a SolverContact,
}

impl<'a> SolverContactView<'a> {
    /// The world-space contact point.
    pub fn point(&self) -> Vect {
        self.raw.point.into()
    }
    /// The distance between the two original contacts points along the contact normal.
    /// If negative, this is measures the penetration depth.
    pub fn dist(&self) -> Real {
        self.raw.dist
    }
    /// The effective friction coefficient at this contact point.
    pub fn friction(&self) -> Real {
        self.raw.friction
    }
    /// The effective restitution coefficient at this contact point.
    pub fn restitution(&self) -> Real {
        self.raw.restitution
    }
    /// The desired tangent relative velocity at the contact point.
    ///
    /// This is set to zero by default. Set to a non-zero value to
    /// simulate, e.g., conveyor belts.
    pub fn tangent_velocity(&self) -> Vect {
        self.raw.tangent_velocity.into()
    }
    /// Whether or not this contact existed during the last timestep.
    pub fn is_new(&self) -> bool {
        self.raw.is_new
    }
}

/// Read-only access to the properties of a contact pair.
pub struct ContactPairView<'a> {
    context_colliders: &'a RapierContextColliders,
    rigidbody_set: &'a RapierRigidBodySet,
    /// The raw contact pair from Rapier.
    pub raw: &'a ContactPair,
}

impl<'a> ContactPairView<'a> {
    /// The first collider involved in this contact pair.
    pub fn collider1(&self) -> Entity {
        self.context_colliders
            .collider_entity(self.raw.collider1)
            .unwrap()
    }

    /// The second collider involved in this contact pair.
    pub fn collider2(&self) -> Entity {
        self.context_colliders
            .collider_entity(self.raw.collider2)
            .unwrap()
    }

    /// The number of contact manifolds detected for this contact pair.
    pub fn manifolds_len(&self) -> usize {
        self.raw.manifolds.len()
    }

    /// Gets the i-th contact manifold.
    pub fn manifold(&self, i: usize) -> Option<ContactManifoldView> {
        self.raw.manifolds.get(i).map(|raw| ContactManifoldView {
            rigidbody_set: self.rigidbody_set,
            raw,
        })
    }

    /// Iterate through all the contact manifolds of this contact pair.
    pub fn manifolds(&self) -> impl ExactSizeIterator<Item = ContactManifoldView> {
        self.raw.manifolds.iter().map(|raw| ContactManifoldView {
            rigidbody_set: self.rigidbody_set,
            raw,
        })
    }

    /// Is there any active contact in this contact pair?
    pub fn has_any_active_contact(&self) -> bool {
        self.raw.has_any_active_contact
    }

    /// Finds the contact with the smallest signed distance.
    ///
    /// If the colliders involved in this contact pair are penetrating, then
    /// this returns the contact with the largest penetration depth.
    ///
    /// Returns a reference to the contact, as well as the contact manifold
    /// it is part of.
    pub fn find_deepest_contact(&self) -> Option<(ContactManifoldView, ContactView)> {
        self.raw.find_deepest_contact().map(|(manifold, contact)| {
            (
                ContactManifoldView {
                    rigidbody_set: self.rigidbody_set,
                    raw: manifold,
                },
                ContactView { raw: contact },
            )
        })
    }
}
