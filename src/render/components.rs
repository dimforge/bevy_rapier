use bevy::prelude::Color;
use bevy::prelude::*;
/// The desired render color of a Rapier collider.
#[derive(Copy, Clone,Component)]
pub struct ColliderDebugRender {
    pub color: Color,
}

pub const DEFAULT_COLOR: Color = Color::BEIGE;
pub const DEFAULT_PALETTE: [Color; 3] = [
    Color::rgb(
        0x98 as f32 / 255.0,
        0xC1 as f32 / 255.0,
        0xD9 as f32 / 255.0,
    ),
    Color::rgb(
        0x05 as f32 / 255.0,
        0x3C as f32 / 255.0,
        0x5E as f32 / 255.0,
    ),
    Color::rgb(
        0x1F as f32 / 255.0,
        0x7A as f32 / 255.0,
        0x8C as f32 / 255.0,
    ),
];

impl Default for ColliderDebugRender {
    fn default() -> Self {
        DEFAULT_COLOR.into()
    }
}

impl From<Color> for ColliderDebugRender {
    fn from(color: Color) -> Self {
        ColliderDebugRender { color }
    }
}

impl ColliderDebugRender {
    pub fn with_id(i: usize) -> Self {
        DEFAULT_PALETTE[i % DEFAULT_PALETTE.len()].into()
    }
}
