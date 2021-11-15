mod visibility;
pub use self::visibility::toggle_visibility;

mod collider;
pub use self::collider::spawn_debug_colliders;

mod render_pass;
pub use self::render_pass::toggle_render_pass;

mod path;
pub use self::path::{
    spawn_debug_paths,
    update_path_mesh
};

mod position;
pub use self::position::spawn_debug_positions;
