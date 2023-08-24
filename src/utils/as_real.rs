
use bevy::math::*;

/// Convert value into the precision the compilation is using.
pub trait AsReal {
    type Real;
    fn as_real(self) -> Self::Real;
}

/// Convert value into single precision floating point.
pub trait AsSingle {
    type Single;
    fn as_single(self) -> Self::Single;
}

macro_rules! as_real_self {
    ($ty:ty) => {
        impl AsReal for $ty {
            type Real = $ty;
            fn as_real(self) -> Self::Real {
                self
            }
        }
    }
}

macro_rules! as_single_self {
    ($ty:ty) => {
        impl AsSingle for $ty {
            type Single = $ty;
            fn as_single(self) -> Self::Single {
                self
            }
        }
    }
}

as_single_self!(f32);
as_single_self!(Vec2);
as_single_self!(Vec3);
as_single_self!(Vec3A);
as_single_self!(Vec4);
as_single_self!(Quat);

impl AsSingle for f64 {
    type Single = f32;
    fn as_single(self) -> Self::Single {
        self as f32
    }
}

impl AsSingle for DVec2 {
    type Single = Vec2;
    fn as_single(self) -> Self::Single {
        self.as_vec2()
    }
}

impl AsSingle for DVec3 {
    type Single = Vec3;
    fn as_single(self) -> Self::Single {
        self.as_vec3()
    }
}

impl AsSingle for DVec4 {
    type Single = Vec4;
    fn as_single(self) -> Self::Single {
        self.as_vec4()
    }
}

impl AsSingle for DQuat {
    type Single = Quat;
    fn as_single(self) -> Self::Single {
        self.as_f32()
    }
}

#[cfg(feature = "f32")]
mod real {
    use bevy::math::*;
    use super::{AsReal, AsSingle};

    as_real_self!(f32);
    as_real_self!(Vec2);
    as_real_self!(Vec3);
    as_real_self!(Vec3A);
    as_real_self!(Vec4);
    as_real_self!(Quat);

    impl AsReal for f64 {
        type Real = f32;
        fn as_real(self) -> Self::Real {
            self.as_single()
        }
    }

    impl AsReal for DVec2 {
        type Real = Vec2;
        fn as_real(self) -> Self::Real {
            self.as_single()
        }
    }

    impl AsReal for DVec3 {
        type Real = Vec3;
        fn as_real(self) -> Self::Real {
            self.as_single()
        }
    }

    impl AsReal for DVec4 {
        type Real = Vec4;
        fn as_real(self) -> Self::Real {
            self.as_single()
        }
    }

    impl AsReal for DQuat {
        type Real = Quat;
        fn as_real(self) -> Self::Real {
            self.as_single()
        }
    }
}

#[cfg(feature = "f64")]
mod real {
    use bevy::math::*;
    use super::AsReal;

    as_real_self!(f64);
    as_real_self!(DVec2);
    as_real_self!(DVec3);
    as_real_self!(DVec4);
    as_real_self!(DQuat);

    impl AsReal for f32 {
        type Real = f64;
        fn as_real(self) -> Self::Real {
            self as f64
        }
    }

    impl AsReal for Vec2 {
        type Real = DVec2;
        fn as_real(self) -> Self::Real {
            self.as_dvec2()
        }
    }

    impl AsReal for Vec3 {
        type Real = DVec3;
        fn as_real(self) -> Self::Real {
            self.as_dvec3()
        }
    }

    impl AsReal for Vec3A {
        type Real = DVec3;
        fn as_real(self) -> Self::Real {
            self.as_dvec3()
        }
    }

    impl AsReal for Vec4 {
        type Real = DVec4;
        fn as_real(self) -> Self::Real {
            self.as_dvec4()
        }
    }

    impl AsReal for Quat {
        type Real = DQuat;
        fn as_real(self) -> Self::Real {
            self.as_f64()
        }
    }
}
