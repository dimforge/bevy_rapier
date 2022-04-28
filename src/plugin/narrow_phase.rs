use crate::math::{Real, Vect};
use crate::plugin::RapierContext;
use bevy::prelude::*;
use rapier::geometry::{Contact, ContactManifold, ContactPair, SolverContact, SolverFlags};

impl RapierContext {
    pub fn contacts_with<'a>(&self, collider: Entity) -> impl Iterator<Item = ContactPairView> {
        self.entity2collider
            .get(&collider)
            .into_iter()
            .flat_map(|h| {
                self.narrow_phase
                    .contacts_with(*h)
                    .map(|raw| ContactPairView { context: self, raw })
            })
    }

    pub fn intersections_with(
        &self,
        collider: Entity,
    ) -> impl Iterator<Item = (Entity, Entity, bool)> + '_ {
        self.entity2collider
            .get(&collider)
            .into_iter()
            .flat_map(|h| {
                self.narrow_phase
                    .intersections_with(*h)
                    .map(|(h1, h2, inter)| {
                        (
                            self.collider_entity(h1).unwrap(),
                            self.collider_entity(h2).unwrap(),
                            inter,
                        )
                    })
            })
    }

    pub fn contact_pair(&self, collider1: Entity, collider2: Entity) -> Option<ContactPairView> {
        let h1 = self.entity2collider.get(&collider1)?;
        let h2 = self.entity2collider.get(&collider2)?;
        self.narrow_phase
            .contact_pair(*h1, *h2)
            .map(|raw| ContactPairView { context: self, raw })
    }

    pub fn intersection_pair(&self, collider1: Entity, collider2: Entity) -> Option<bool> {
        let h1 = self.entity2collider.get(&collider1)?;
        let h2 = self.entity2collider.get(&collider2)?;
        self.narrow_phase.intersection_pair(*h1, *h2)
    }

    pub fn contact_pairs(&self) -> impl Iterator<Item = ContactPairView> {
        self.narrow_phase
            .contact_pairs()
            .map(|raw| ContactPairView { context: self, raw })
    }

    pub fn intersection_pairs(&self) -> impl Iterator<Item = (Entity, Entity, bool)> + '_ {
        self.narrow_phase
            .intersection_pairs()
            .map(|(h1, h2, inter)| {
                (
                    self.collider_entity(h1).unwrap(),
                    self.collider_entity(h2).unwrap(),
                    inter,
                )
            })
    }
}

pub struct ContactManifoldView<'a> {
    context: &'a RapierContext,
    pub raw: &'a ContactManifold,
}

impl<'a> ContactManifoldView<'a> {
    pub fn num_points(&self) -> usize {
        self.raw.points.len()
    }

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
            .and_then(|h| self.context.rigid_body_entity(h))
    }

    /// The second rigid-body involved in this contact manifold.
    pub fn rigid_body2(&self) -> Option<Entity> {
        self.raw
            .data
            .rigid_body2
            .and_then(|h| self.context.rigid_body_entity(h))
    }

    /// Flags used to control some aspects of the constraints solver for this contact manifold.
    pub fn solver_flags(&self) -> SolverFlags {
        self.raw.data.solver_flags
    }

    /// The world-space contact normal shared by all the contact in this contact manifold.
    pub fn normal(&self) -> Vect {
        self.raw.data.normal.into()
    }

    pub fn num_solver_contacts(&self) -> usize {
        self.raw.data.solver_contacts.len()
    }

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

pub struct ContactView<'a> {
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
        self.raw.fid1
    }

    /// The feature ID of the second shape involved in the contact.
    pub fn fid2(&self) -> u32 {
        self.raw.fid2
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
        self.raw.data.tangent_impulse
    }

    /// The friction impulse along the vector orthonormal to the contact normal, applied to the first
    /// collider's rigid-body.
    #[cfg(feature = "dim3")]
    pub fn tangent_impulse(&self) -> [Real; 2] {
        self.raw.data.tangent_impulse.into()
    }
}

pub struct SolverContactView<'a> {
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

pub struct ContactPairView<'a> {
    context: &'a RapierContext,
    pub raw: &'a ContactPair,
}

impl<'a> ContactPairView<'a> {
    pub fn collider1(&self) -> Entity {
        self.context.collider_entity(self.raw.collider1).unwrap()
    }

    pub fn collider2(&self) -> Entity {
        self.context.collider_entity(self.raw.collider2).unwrap()
    }

    pub fn manifolds_len(&self) -> usize {
        self.raw.manifolds.len()
    }

    pub fn manifold(&self, i: usize) -> Option<ContactManifoldView> {
        self.raw.manifolds.get(i).map(|raw| ContactManifoldView {
            context: self.context,
            raw,
        })
    }

    pub fn manifolds(&self) -> impl ExactSizeIterator<Item = ContactManifoldView> {
        self.raw.manifolds.iter().map(|raw| ContactManifoldView {
            context: self.context,
            raw,
        })
    }

    pub fn has_any_active_contacts(&self) -> bool {
        self.has_any_active_contacts()
    }

    pub fn find_deepest_contact(&self) -> Option<(ContactManifoldView, ContactView)> {
        self.raw.find_deepest_contact().map(|(manifold, contact)| {
            (
                ContactManifoldView {
                    context: self.context,
                    raw: manifold,
                },
                ContactView { raw: contact },
            )
        })
    }
}
