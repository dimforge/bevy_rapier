pub use self::generic_joint::*;
pub use self::joint::*;
pub use self::rigid_body::*;

pub use self::fixed_joint::*;
pub use self::prismatic_joint::*;
pub use self::revolute_joint::*;

use bevy::reflect::Reflect;
use rapier::dynamics::CoefficientCombineRule as RapierCoefficientCombineRule;

#[cfg(feature = "dim3")]
pub use self::spherical_joint::*;

mod generic_joint;
mod joint;
mod rigid_body;

mod fixed_joint;
mod prismatic_joint;
mod revolute_joint;

#[cfg(feature = "dim3")]
mod spherical_joint;

/// Rules used to combine two coefficients.
///
/// This is used to determine the effective restitution and
/// friction coefficients for a contact between two colliders.
/// Each collider has its combination rule of type
/// `CoefficientCombineRule`. And the rule
/// actually used is given by `max(first_combine_rule as usize, second_combine_rule as usize)`.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Reflect)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub enum CoefficientCombineRule {
    /// The two coefficients are averaged.
    Average = 0,
    /// The smallest coefficient is chosen.
    Min,
    /// The two coefficients are multiplied.
    Multiply,
    /// The greatest coefficient is chosen.
    Max,
}

impl Default for CoefficientCombineRule {
    fn default() -> Self {
        CoefficientCombineRule::Average
    }
}

impl Into<RapierCoefficientCombineRule> for CoefficientCombineRule {
    fn into(self) -> RapierCoefficientCombineRule {
        match self {
            Self::Average => RapierCoefficientCombineRule::Average,
            Self::Min => RapierCoefficientCombineRule::Min,
            Self::Multiply => RapierCoefficientCombineRule::Multiply,
            Self::Max => RapierCoefficientCombineRule::Max,
        }
    }
}
