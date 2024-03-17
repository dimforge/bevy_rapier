use rapier::geometry::{
    Ball, Capsule, Compound, Cuboid, HalfSpace, HeightField, Polyline, RoundShape, Segment,
    TriMesh, Triangle, TypedShape,
};
#[cfg(feature = "dim3")]
use rapier::geometry::{ConvexPolyhedron, Cylinder};
#[cfg(all(feature = "dim3", feature = "async-collider"))]
use {
    bevy::prelude::*,
    bevy::render::mesh::{Indices, VertexAttributeValues},
};
#[cfg(feature = "dim2")]
use {na::DVector, rapier::geometry::ConvexPolygon};

#[cfg(feature = "dim3")]
use rapier::geometry::Cone;

use rapier::prelude::{
    FeatureId, Point, PointProjection, Ray, RayIntersection, SharedShape, Vector, DIM,
};

use super::get_snapped_scale;
#[cfg(all(feature = "dim3", feature = "async-collider"))]
use crate::geometry::ComputedColliderShape;
use crate::geometry::{Collider, TriMeshFlags, VHACDParameters};
use crate::math::{Isometry, Real, Vect};
use rapier::parry::either::Either;

impl Collider {
    /// The scaling factor that was applied to this collider.
    pub fn scale(&self) -> Vect {
        self.scale
    }

    /// This replaces the unscaled version of this collider by its scaled version,
    /// and resets `self.scale()` to `1.0`.
    pub fn promote_scaled_shape(&mut self) {
        self.unscaled = self.raw.clone();
        self.scale = Vect::ONE;
    }

    /// Initialize a new collider with a compound shape.
    pub fn compound(shapes: Vec<(Isometry, Collider)>) -> Self {
        let shapes = shapes.into_iter().map(|(iso, s)| (iso, s.raw)).collect();
        SharedShape::compound(shapes).into()
    }

    /// Initialize a new collider with a ball shape defined by its radius.
    pub fn ball(radius: Real) -> Self {
        SharedShape::ball(radius).into()
    }

    /// Initialize a new collider build with a half-space shape defined by the outward normal
    /// of its planar boundary.
    pub fn halfspace(outward_normal: Vect) -> Option<Self> {
        let normalized = outward_normal.try_normalize()?;
        Some(SharedShape::halfspace(normalized).into())
    }

    /// Initialize a new collider with a cylindrical shape defined by its half-height
    /// (along along the y axis) and its radius.
    #[cfg(feature = "dim3")]
    pub fn cylinder(half_height: Real, radius: Real) -> Self {
        SharedShape::cylinder(half_height, radius).into()
    }

    /// Initialize a new collider with a rounded cylindrical shape defined by its half-height
    /// (along along the y axis), its radius, and its roundedness (the
    /// radius of the sphere used for dilating the cylinder).
    #[cfg(feature = "dim3")]
    pub fn round_cylinder(half_height: Real, radius: Real, border_radius: Real) -> Self {
        SharedShape::round_cylinder(half_height, radius, border_radius).into()
    }

    /// Initialize a new collider with a cone shape defined by its half-height
    /// (along along the y axis) and its basis radius.
    #[cfg(feature = "dim3")]
    pub fn cone(half_height: Real, radius: Real) -> Self {
        SharedShape::cone(half_height, radius).into()
    }

    /// Initialize a new collider with a rounded cone shape defined by its half-height
    /// (along along the y axis), its radius, and its roundedness (the
    /// radius of the sphere used for dilating the cylinder).
    #[cfg(feature = "dim3")]
    pub fn round_cone(half_height: Real, radius: Real, border_radius: Real) -> Self {
        SharedShape::round_cone(half_height, radius, border_radius).into()
    }

    /// Initialize a new collider with a cuboid shape defined by its half-extents.
    #[cfg(feature = "dim2")]
    pub fn cuboid(half_x: Real, half_y: Real) -> Self {
        SharedShape::cuboid(half_x, half_y).into()
    }

    /// Initialize a new collider with a round cuboid shape defined by its half-extents
    /// and border radius.
    #[cfg(feature = "dim2")]
    pub fn round_cuboid(half_x: Real, half_y: Real, border_radius: Real) -> Self {
        SharedShape::round_cuboid(half_x, half_y, border_radius).into()
    }

    /// Initialize a new collider with a capsule shape.
    pub fn capsule(start: Vect, end: Vect, radius: Real) -> Self {
        SharedShape::capsule(start, end, radius).into()
    }

    /// Initialize a new collider with a capsule shape aligned with the `x` axis.
    pub fn capsule_x(half_height: Real, radius: Real) -> Self {
        let p = Point::from(Vector::X * half_height);
        SharedShape::capsule(-p, p, radius).into()
    }

    /// Initialize a new collider with a capsule shape aligned with the `y` axis.
    pub fn capsule_y(half_height: Real, radius: Real) -> Self {
        let p = Point::from(Vector::Y * half_height);
        SharedShape::capsule(-p, p, radius).into()
    }

    /// Initialize a new collider with a capsule shape aligned with the `z` axis.
    #[cfg(feature = "dim3")]
    pub fn capsule_z(half_height: Real, radius: Real) -> Self {
        let p = Point::from(Vector::Z * half_height);
        SharedShape::capsule(-p, p, radius).into()
    }

    /// Initialize a new collider with a cuboid shape defined by its half-extents.
    #[cfg(feature = "dim3")]
    pub fn cuboid(hx: Real, hy: Real, hz: Real) -> Self {
        SharedShape::cuboid(hx, hy, hz).into()
    }

    /// Initialize a new collider with a round cuboid shape defined by its half-extents
    /// and border radius.
    #[cfg(feature = "dim3")]
    pub fn round_cuboid(half_x: Real, half_y: Real, half_z: Real, border_radius: Real) -> Self {
        SharedShape::round_cuboid(half_x, half_y, half_z, border_radius).into()
    }

    /// Initializes a collider with a segment shape.
    pub fn segment(a: Vect, b: Vect) -> Self {
        SharedShape::segment(a, b).into()
    }

    /// Initializes a collider with a triangle shape.
    pub fn triangle(a: Vect, b: Vect, c: Vect) -> Self {
        SharedShape::triangle(a, b, c).into()
    }

    /// Initializes a collider with a triangle shape with round corners.
    pub fn round_triangle(a: Vect, b: Vect, c: Vect, border_radius: Real) -> Self {
        SharedShape::round_triangle(a, b, c, border_radius).into()
    }

    /// Initializes a collider with a polyline shape defined by its vertex and index buffers.
    pub fn polyline(vertices: Vec<Vect>, indices: Option<Vec<[u32; 2]>>) -> Self {
        SharedShape::polyline(vertices, indices).into()
    }

    /// Initializes a collider with a triangle mesh shape defined by its vertex and index buffers.
    pub fn trimesh(vertices: Vec<Vect>, indices: Vec<[u32; 3]>) -> Self {
        SharedShape::trimesh(vertices, indices).into()
    }

    /// Initializes a collider with a triangle mesh shape defined by its vertex and index buffers, and flags
    /// controlling its pre-processing.
    pub fn trimesh_with_flags(
        vertices: Vec<Vect>,
        indices: Vec<[u32; 3]>,
        flags: TriMeshFlags,
    ) -> Self {
        SharedShape::trimesh_with_flags(vertices, indices, flags).into()
    }

    /// Initializes a collider with a Bevy Mesh.
    ///
    /// Returns `None` if the index buffer or vertex buffer of the mesh are in an incompatible format.
    #[cfg(all(feature = "dim3", feature = "async-collider"))]
    pub fn from_bevy_mesh(mesh: &Mesh, collider_shape: &ComputedColliderShape) -> Option<Self> {
        let Some((vtx, idx)) = extract_mesh_vertices_indices(mesh) else {
            return None;
        };
        match collider_shape {
            ComputedColliderShape::TriMesh => Some(
                SharedShape::trimesh_with_flags(vtx, idx, TriMeshFlags::MERGE_DUPLICATE_VERTICES)
                    .into(),
            ),
            ComputedColliderShape::ConvexHull => {
                SharedShape::convex_hull(&vtx).map(|shape| shape.into())
            }
            ComputedColliderShape::ConvexDecomposition(params) => {
                Some(SharedShape::convex_decomposition_with_params(&vtx, &idx, params).into())
            }
        }
    }

    /// Initializes a collider with a compound shape obtained from the decomposition of
    /// the given trimesh (in 3D) or polyline (in 2D) into convex parts.
    pub fn convex_decomposition(vertices: &[Vect], indices: &[[u32; DIM]]) -> Self {
        let vertices: Vec<_> = vertices.iter().map(|v| (*v).into()).collect();
        SharedShape::convex_decomposition(&vertices, indices).into()
    }

    /// Initializes a collider with a compound shape obtained from the decomposition of
    /// the given trimesh (in 3D) or polyline (in 2D) into convex parts dilated with round corners.
    pub fn round_convex_decomposition(
        vertices: &[Vect],
        indices: &[[u32; DIM]],
        border_radius: Real,
    ) -> Self {
        let vertices: Vec<_> = vertices.iter().map(|v| (*v).into()).collect();
        SharedShape::round_convex_decomposition(&vertices, indices, border_radius).into()
    }

    /// Initializes a collider with a compound shape obtained from the decomposition of
    /// the given trimesh (in 3D) or polyline (in 2D) into convex parts.
    pub fn convex_decomposition_with_params(
        vertices: &[Vect],
        indices: &[[u32; DIM]],
        params: &VHACDParameters,
    ) -> Self {
        let vertices: Vec<_> = vertices.iter().map(|v| (*v).into()).collect();
        SharedShape::convex_decomposition_with_params(&vertices, indices, params).into()
    }

    /// Initializes a collider with a compound shape obtained from the decomposition of
    /// the given trimesh (in 3D) or polyline (in 2D) into convex parts dilated with round corners.
    pub fn round_convex_decomposition_with_params(
        vertices: &[Vect],
        indices: &[[u32; DIM]],
        params: &VHACDParameters,
        border_radius: Real,
    ) -> Self {
        let vertices: Vec<_> = vertices.iter().map(|v| (*v).into()).collect();
        SharedShape::round_convex_decomposition_with_params(
            &vertices,
            indices,
            params,
            border_radius,
        )
        .into()
    }

    /// Initializes a new collider with a 2D convex polygon or 3D convex polyhedron
    /// obtained after computing the convex-hull of the given points.
    pub fn convex_hull(points: &[Vect]) -> Option<Self> {
        let points: Vec<_> = points.iter().map(|v| (*v).into()).collect();
        SharedShape::convex_hull(&points).map(Into::into)
    }

    /// Initializes a new collider with a round 2D convex polygon or 3D convex polyhedron
    /// obtained after computing the convex-hull of the given points. The shape is dilated
    /// by a sphere of radius `border_radius`.
    pub fn round_convex_hull(points: &[Vect], border_radius: Real) -> Option<Self> {
        let points: Vec<_> = points.iter().map(|v| (*v).into()).collect();
        SharedShape::round_convex_hull(&points, border_radius).map(Into::into)
    }

    /// Creates a new collider that is a convex polygon formed by the
    /// given polyline assumed to be convex (no convex-hull will be automatically
    /// computed).
    #[cfg(feature = "dim2")]
    pub fn convex_polyline(points: Vec<Vect>) -> Option<Self> {
        let points = points.into_iter().map(|v| v.into()).collect();
        SharedShape::convex_polyline(points).map(Into::into)
    }

    /// Creates a new collider that is a round convex polygon formed by the
    /// given polyline assumed to be convex (no convex-hull will be automatically
    /// computed). The polygon shape is dilated by a sphere of radius `border_radius`.
    #[cfg(feature = "dim2")]
    pub fn round_convex_polyline(points: Vec<Vect>, border_radius: Real) -> Option<Self> {
        let points = points.into_iter().map(|v| v.into()).collect();
        SharedShape::round_convex_polyline(points, border_radius).map(Into::into)
    }

    /// Creates a new collider that is a convex polyhedron formed by the
    /// given triangle-mesh assumed to be convex (no convex-hull will be automatically
    /// computed).
    #[cfg(feature = "dim3")]
    pub fn convex_mesh(points: Vec<Vect>, indices: &[[u32; 3]]) -> Option<Self> {
        let points = points.into_iter().map(|v| v.into()).collect();
        SharedShape::convex_mesh(points, indices).map(Into::into)
    }

    /// Creates a new collider that is a round convex polyhedron formed by the
    /// given triangle-mesh assumed to be convex (no convex-hull will be automatically
    /// computed). The triangle mesh shape is dilated by a sphere of radius `border_radius`.
    #[cfg(feature = "dim3")]
    pub fn round_convex_mesh(
        points: Vec<Vect>,
        indices: &[[u32; 3]],
        border_radius: Real,
    ) -> Option<Self> {
        let points = points.into_iter().map(|v| v.into()).collect();
        SharedShape::round_convex_mesh(points, indices, border_radius).map(Into::into)
    }

    /// Initializes a collider with a heightfield shape defined by its set of height and a scale
    /// factor along each coordinate axis.
    #[cfg(feature = "dim2")]
    pub fn heightfield(heights: Vec<Real>, scale: Vect) -> Self {
        SharedShape::heightfield(DVector::from_vec(heights), scale.into()).into()
    }

    /// Initializes a collider with a heightfield shape defined by its set of height (in
    /// column-major format) and a scale factor along each coordinate axis.
    #[cfg(feature = "dim3")]
    pub fn heightfield(heights: Vec<Real>, num_rows: usize, num_cols: usize, scale: Vect) -> Self {
        assert_eq!(
            heights.len(),
            num_rows * num_cols,
            "Invalid number of heights provided."
        );
        let heights = rapier::na::DMatrix::from_vec(num_rows, num_cols, heights);
        SharedShape::heightfield(heights, scale.into()).into()
    }

    /// Takes a strongly typed reference of this collider.
    pub fn as_typed_shape(&self) -> TypedShape {
        self.raw.as_typed_shape()
    }

    /// Takes a strongly typed reference of the unscaled version of this collider.
    pub fn as_unscaled_typed_shape(&self) -> TypedShape {
        self.unscaled.as_typed_shape()
    }

    /// Downcast this collider to a ball, if it is one.
    pub fn as_ball(&self) -> Option<&Ball> {
        self.raw.as_ball()
    }

    /// Downcast this collider to a cuboid, if it is one.
    pub fn as_cuboid(&self) -> Option<&Cuboid> {
        self.raw.as_cuboid()
    }

    /// Downcast this collider to a capsule, if it is one.
    pub fn as_capsule(&self) -> Option<&Capsule> {
        self.raw.as_capsule()
    }

    /// Downcast this collider to a segment, if it is one.
    pub fn as_segment(&self) -> Option<&Segment> {
        self.raw.as_segment()
    }

    /// Downcast this collider to a triangle, if it is one.
    pub fn as_triangle(&self) -> Option<&Triangle> {
        self.raw.as_triangle()
    }

    /// Downcast this collider to a triangle mesh, if it is one.
    pub fn as_trimesh(&self) -> Option<&TriMesh> {
        self.raw.as_trimesh()
    }

    /// Downcast this collider to a polyline, if it is one.
    pub fn as_polyline(&self) -> Option<&Polyline> {
        self.raw.as_polyline()
    }

    /// Downcast this collider to a half-space, if it is one.
    pub fn as_halfspace(&self) -> Option<&HalfSpace> {
        self.raw.as_halfspace()
    }

    /// Downcast this collider to a heightfield, if it is one.
    pub fn as_heightfield(&self) -> Option<&HeightField> {
        self.raw.as_heightfield()
    }

    /// Downcast this collider to a compound shape, if it is one.
    pub fn as_compound(&self) -> Option<&Compound> {
        self.raw.as_compound()
    }

    /// Downcast this collider to a convex polygon, if it is one.
    #[cfg(feature = "dim2")]
    pub fn as_convex_polygon(&self) -> Option<&ConvexPolygon> {
        self.raw.as_convex_polygon()
    }

    /// Downcast this collider to a convex polyhedron, if it is one.
    #[cfg(feature = "dim3")]
    pub fn as_convex_polyhedron(&self) -> Option<&ConvexPolyhedron> {
        self.raw.as_convex_polyhedron()
    }

    /// Downcast this collider to a cylinder, if it is one.
    #[cfg(feature = "dim3")]
    pub fn as_cylinder(&self) -> Option<&Cylinder> {
        self.raw.as_cylinder()
    }

    /// Downcast this collider to a cone, if it is one.
    #[cfg(feature = "dim3")]
    pub fn as_cone(&self) -> Option<&Cone> {
        self.raw.as_cone()
    }

    /// Downcast this collider to a mutable ball, if it is one.
    pub fn as_ball_mut(&mut self) -> Option<&mut Ball> {
        self.raw.make_mut().as_ball_mut()
    }

    /// Downcast this collider to a mutable cuboid, if it is one.
    pub fn as_cuboid_mut(&mut self) -> Option<&mut Cuboid> {
        self.raw.make_mut().as_cuboid_mut()
    }

    /// Downcast this collider to a mutable capsule, if it is one.
    pub fn as_capsule_mut(&mut self) -> Option<&mut Capsule> {
        self.raw.make_mut().as_capsule_mut()
    }

    /// Downcast this collider to a mutable segment, if it is one.
    pub fn as_segment_mut(&mut self) -> Option<&mut Segment> {
        self.raw.make_mut().as_segment_mut()
    }

    /// Downcast this collider to a mutable triangle, if it is one.
    pub fn as_triangle_mut(&mut self) -> Option<&mut Triangle> {
        self.raw.make_mut().as_triangle_mut()
    }

    /// Downcast this collider to a mutable triangle mesh, if it is one.
    pub fn as_trimesh_mut(&mut self) -> Option<&mut TriMesh> {
        self.raw.make_mut().as_trimesh_mut()
    }

    /// Downcast this collider to a mutable polyline, if it is one.
    pub fn as_polyline_mut(&mut self) -> Option<&mut Polyline> {
        self.raw.make_mut().as_polyline_mut()
    }

    /// Downcast this collider to a mutable half-space, if it is one.
    pub fn as_halfspace_mut(&mut self) -> Option<&mut HalfSpace> {
        self.raw.make_mut().as_halfspace_mut()
    }

    /// Downcast this collider to a mutable heightfield, if it is one.
    pub fn as_heightfield_mut(&mut self) -> Option<&mut HeightField> {
        self.raw.make_mut().as_heightfield_mut()
    }

    /// Downcast this collider to a mutable cylinder, if it is one.
    #[cfg(feature = "dim3")]
    pub fn as_cylinder_mut(&mut self) -> Option<&mut Cylinder> {
        self.raw.make_mut().as_cylinder_mut()
    }

    /// Downcast this collider to a mutable cone, if it is one.
    #[cfg(feature = "dim3")]
    pub fn as_cone_mut(&mut self) -> Option<&mut Cone> {
        self.raw.make_mut().as_cone_mut()
    }

    /// Set the scaling factor of this shape.
    ///
    /// If the scaling factor is non-uniform, and the scaled shape can’t be
    /// represented as a supported smooth shape (for example scalling a Ball
    /// with a non-uniform scale results in an ellipse which isn’t supported),
    /// the shape is approximated by a convex polygon/convex polyhedron using
    /// `num_subdivisions` subdivisions.
    pub fn set_scale(&mut self, scale: Vect, num_subdivisions: u32) {
        let scale = get_snapped_scale(scale);

        if scale == self.scale {
            // Nothing to do.
            return;
        }

        if scale == Vect::ONE {
            // Trivial case.
            self.raw = self.unscaled.clone();
            self.scale = Vect::ONE;
            return;
        }

        if let Some(scaled) =
            scale_typed_shape(self.as_unscaled_typed_shape(), scale, num_subdivisions)
        {
            self.raw = scaled;
            self.scale = scale;
        } else {
            log::error!("Failed to create the scaled convex hull geometry.");
        }
    }

    /// Projects a point on `self`, unless the projection lies further than the given max distance.
    ///
    /// The point is assumed to be expressed in the local-space of `self`.
    pub fn project_local_point_with_max_dist(
        &self,
        point: Vect,
        solid: bool,
        max_dist: Real,
    ) -> Option<PointProjection> {
        self.raw
            .project_local_point_with_max_dist(&point.into(), solid, max_dist)
            .map(Into::into)
    }

    /// Projects a point on `self` transformed by `collider_pos`, unless the projection lies further than the given max distance.
    pub fn project_point_with_max_dist(
        &self,
        collider_pos: Isometry,
        point: Vect,
        solid: bool,
        max_dist: Real,
    ) -> Option<PointProjection> {
        self.raw
            .project_point_with_max_dist(&collider_pos, &point.into(), solid, max_dist)
            .map(Into::into)
    }

    /// Projects a point on `self`.
    ///
    /// The point is assumed to be expressed in the local-space of `self`.
    pub fn project_local_point(&self, point: Vect, solid: bool) -> PointProjection {
        self.raw.project_local_point(&point.into(), solid).into()
    }

    /// Projects a point on the boundary of `self` and returns the id of the
    /// feature the point was projected on.
    pub fn project_local_point_and_get_feature(&self, point: Vect) -> (PointProjection, FeatureId) {
        let (proj, feat) = self.raw.project_local_point_and_get_feature(&point.into());
        (proj.into(), feat)
    }

    /// Computes the minimal distance between a point and `self`.
    pub fn distance_to_local_point(&self, point: Vect, solid: bool) -> Real {
        self.raw.distance_to_local_point(&point.into(), solid)
    }

    /// Tests if the given point is inside of `self`.
    pub fn contains_local_point(&self, point: Vect) -> bool {
        self.raw.contains_local_point(&point.into())
    }

    /// Projects a point on `self` transformed by `collider_pos`.
    pub fn project_point(
        &self,
        collider_pos: Isometry,
        point: Vect,
        solid: bool,
    ) -> PointProjection {
        self.raw
            .project_point(&collider_pos, &point.into(), solid)
            .into()
    }

    /// Computes the minimal distance between a point and `self` transformed by `collider_pos`.
    #[inline]
    pub fn distance_to_point(&self, collider_pos: Isometry, point: Vect, solid: bool) -> Real {
        self.raw
            .distance_to_point(&collider_pos, &point.into(), solid)
    }

    /// Projects a point on the boundary of `self` transformed by `collider_pos` and returns the id of the
    /// feature the point was projected on.
    pub fn project_point_and_get_feature(
        &self,
        collider_pos: Isometry,
        point: Vect,
    ) -> (PointProjection, FeatureId) {
        let (proj, feat) = self
            .raw
            .project_point_and_get_feature(&collider_pos, &point.into());
        (proj.into(), feat)
    }

    /// Tests if the given point is inside of `self` transformed by `collider_&os`.
    pub fn contains_point(&self, collider_pos: Isometry, point: Vect) -> bool {
        self.raw.contains_point(&collider_pos, &point.into())
    }

    /// Computes the time of impact between this transform shape and a ray.
    pub fn cast_local_ray(&self, ray: Ray, max_time_of_impact: Real, solid: bool) -> Option<Real> {
        self.raw.cast_local_ray(&ray, max_time_of_impact, solid)
    }

    /// Computes the time of impact, and normal between this transformed shape and a ray.
    pub fn cast_local_ray_and_get_normal(
        &self,
        ray: Ray,
        max_time_of_impact: Real,
        solid: bool,
    ) -> Option<RayIntersection> {
        self.raw
            .cast_local_ray_and_get_normal(&ray, max_time_of_impact, solid)
    }

    /// Tests whether a ray intersects this transformed shape.
    pub fn intersects_local_ray(&self, ray: Ray, max_time_of_impact: Real) -> bool {
        self.raw.intersects_local_ray(&ray, max_time_of_impact)
    }

    /// Computes the time of impact between this transformed shape and a ray.
    pub fn cast_ray(
        &self,
        collider_pos: Isometry,
        ray: Ray,
        max_time_of_impact: Real,
        solid: bool,
    ) -> Option<Real> {
        self.raw
            .cast_ray(&collider_pos, &ray, max_time_of_impact, solid)
    }

    /// Computes the time of impact, and normal between this transformed shape and a ray.
    pub fn cast_ray_and_get_normal(
        &self,
        collider_pos: Isometry,
        ray: Ray,
        max_time_of_impact: Real,
        solid: bool,
    ) -> Option<RayIntersection> {
        self.raw
            .cast_ray_and_get_normal(&collider_pos, &ray, max_time_of_impact, solid)
    }

    /// Tests whether a ray intersects this transformed shape.
    pub fn intersects_ray(
        &self,
        collider_pos: Isometry,
        ray: Ray,
        max_time_of_impact: Real,
    ) -> bool {
        self.raw
            .intersects_ray(&collider_pos, &ray, max_time_of_impact)
    }
}

impl Default for Collider {
    fn default() -> Self {
        Self::ball(0.5)
    }
}

#[cfg(all(feature = "dim3", feature = "async-collider"))]
#[allow(clippy::type_complexity)]
fn extract_mesh_vertices_indices(mesh: &Mesh) -> Option<(Vec<Point>, Vec<[u32; 3]>)> {
    let vertices = mesh.attribute(Mesh::ATTRIBUTE_POSITION)?;
    let indices = mesh.indices()?;

    let vtx: Vec<_> = match vertices {
        VertexAttributeValues::Float32(vtx) => Some(
            vtx.chunks(3)
                .map(|v| Point::new(v[0] as Real, v[1] as Real, v[2] as Real))
                .collect(),
        ),
        VertexAttributeValues::Float32x3(vtx) => Some(
            vtx.iter()
                .map(|v| Point::new(v[0] as Real, v[1] as Real, v[2] as Real))
                .collect(),
        ),
        _ => None,
    }?;

    let idx = match indices {
        Indices::U16(idx) => idx
            .chunks_exact(3)
            .map(|i| [i[0] as u32, i[1] as u32, i[2] as u32])
            .collect(),
        Indices::U32(idx) => idx.chunks_exact(3).map(|i| [i[0], i[1], i[2]]).collect(),
    };

    Some((vtx, idx))
}

/// Compute the scaled version of thi given shape.
///
/// If the scaling factor is non-uniform, and the scaled shape can’t be
/// represented as a supported smooth shape (for example scalling a Ball
/// with a non-uniform scale results in an ellipse which isn’t supported),
/// the shape is approximated by a convex polygon/convex polyhedron using
// TODO: would this be worth upstreaming to parry?
pub fn scale_typed_shape(
    shape: TypedShape,
    scale: Vect,
    num_subdivisions: u32,
) -> Option<SharedShape> {
    let result = match shape {
        TypedShape::Custom(_) => return None,
        TypedShape::Cuboid(s) => SharedShape::new(s.scaled(&scale.into())),
        TypedShape::RoundCuboid(s) => SharedShape::new(RoundShape {
            border_radius: s.border_radius,
            inner_shape: s.inner_shape.scaled(&scale.into()),
        }),
        TypedShape::Capsule(c) => match c.scaled(&scale.into(), num_subdivisions)? {
            Either::Left(b) => SharedShape::new(b),
            Either::Right(b) => SharedShape::new(b),
        },
        TypedShape::Ball(b) => match b.scaled(&scale.into(), num_subdivisions)? {
            Either::Left(b) => SharedShape::new(b),
            Either::Right(b) => SharedShape::new(b),
        },
        TypedShape::Segment(s) => SharedShape::new(s.scaled(&scale.into())),
        // TypedShape::RoundSegment(s) => SharedShape::new(RoundShape {
        //     border_radius: s.border_radius,
        //     inner_shape: s.inner_shape.scaled(&scale.into()),
        // }),
        TypedShape::Triangle(t) => SharedShape::new(t.scaled(&scale.into())),
        TypedShape::RoundTriangle(t) => SharedShape::new(RoundShape {
            border_radius: t.border_radius,
            inner_shape: t.inner_shape.scaled(&scale.into()),
        }),
        TypedShape::TriMesh(t) => SharedShape::new(t.clone().scaled(&scale.into())),
        TypedShape::Polyline(p) => SharedShape::new(p.clone().scaled(&scale.into())),
        TypedShape::HalfSpace(h) => SharedShape::new(h.scaled(&scale.into())?),
        TypedShape::HeightField(h) => SharedShape::new(h.clone().scaled(&scale.into())),
        #[cfg(feature = "dim2")]
        TypedShape::ConvexPolygon(cp) => SharedShape::new(cp.clone().scaled(&scale.into())?),
        #[cfg(feature = "dim2")]
        TypedShape::RoundConvexPolygon(cp) => {
            let scaled = cp.inner_shape.clone().scaled(&scale.into())?;
            SharedShape::new(RoundShape {
                border_radius: cp.border_radius,
                inner_shape: scaled,
            })
        }
        #[cfg(feature = "dim3")]
        TypedShape::ConvexPolyhedron(cp) => SharedShape::new(cp.clone().scaled(&scale.into())?),
        #[cfg(feature = "dim3")]
        TypedShape::RoundConvexPolyhedron(cp) => {
            let scaled = cp.clone().inner_shape.scaled(&scale.into())?;
            SharedShape::new(RoundShape {
                border_radius: cp.border_radius,
                inner_shape: scaled,
            })
        }
        #[cfg(feature = "dim3")]
        TypedShape::Cylinder(c) => match c.scaled(&scale.into(), num_subdivisions)? {
            Either::Left(b) => SharedShape::new(b),
            Either::Right(b) => SharedShape::new(b),
        },
        #[cfg(feature = "dim3")]
        TypedShape::RoundCylinder(c) => {
            match c.inner_shape.scaled(&scale.into(), num_subdivisions)? {
                Either::Left(scaled) => SharedShape::new(RoundShape {
                    border_radius: c.border_radius,
                    inner_shape: scaled,
                }),
                Either::Right(scaled) => SharedShape::new(RoundShape {
                    border_radius: c.border_radius,
                    inner_shape: scaled,
                }),
            }
        }
        #[cfg(feature = "dim3")]
        TypedShape::Cone(c) => match c.scaled(&scale.into(), num_subdivisions)? {
            Either::Left(b) => SharedShape::new(b),
            Either::Right(b) => SharedShape::new(b),
        },
        #[cfg(feature = "dim3")]
        TypedShape::RoundCone(c) => match c.inner_shape.scaled(&scale.into(), num_subdivisions)? {
            Either::Left(scaled) => SharedShape::new(RoundShape {
                border_radius: c.border_radius,
                inner_shape: scaled,
            }),
            Either::Right(scaled) => SharedShape::new(RoundShape {
                border_radius: c.border_radius,
                inner_shape: scaled,
            }),
        },
        TypedShape::Compound(c) => {
            let mut scaled = Vec::with_capacity(c.shapes().len());

            for (pos, shape) in c.shapes() {
                let mut scaled_pos = *pos;
                scaled_pos.translation *= scale;

                if let Some(scaled_shape) =
                    scale_typed_shape(shape.as_typed_shape(), scale, num_subdivisions)
                {
                    scaled.push((scaled_pos, scaled_shape));
                }
            }
            SharedShape::compound(scaled)
        }
    };

    Some(result)
}
