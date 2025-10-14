pub(crate) use self::events::EventQueue;
pub use self::events::{CollisionMessage, ContactForceMessage};

#[deprecated(
    since = "0.32.0",
    note = "CollisionEvent has been renamed to CollisionMessage for consistency with Bevy 0.17 naming conventions. "
)]
pub use CollisionMessage as CollisionEvent;

#[deprecated(
    since = "0.32.0",
    note = "ContactForceEvent has been renamed to ContactForceMessage for consistency with Bevy 0.17 naming conventions. "
)]
pub use ContactForceMessage as ContactForceEvent;

pub(crate) use self::physics_hooks::BevyPhysicsHooksAdapter;
pub use self::physics_hooks::{
    BevyPhysicsHooks, ContactModificationContextView, PairFilterContextView,
};
pub use query_filter::{QueryFilter, QueryFilterFlags};

mod events;
mod physics_hooks;
mod query_filter;
