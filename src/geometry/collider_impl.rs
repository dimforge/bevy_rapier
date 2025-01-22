#[cfg(feature = "dim2")]
use na::DVector;
#[cfg(all(feature = "dim3", feature = "async-collider"))]
use {
    bevy::prelude::*,
    bevy::render::mesh::{Indices, VertexAttributeValues},
};

use rapier::prelude::{FeatureId, Point, Ray, SharedShape, Vector, DIM};

use super::{get_snapped_scale, shape_views::*};
#[cfg(all(feature = "dim3", feature = "async-collider"))]
use crate::geometry::ComputedColliderShape;
use crate::geometry::{Collider, PointProjection, RayIntersection, TriMeshFlags, VHACDParameters};
use crate::math::{Real, Rot, Vect};

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
    pub fn compound(shapes: Vec<(Vect, Rot, Collider)>) -> Self {
        let shapes = shapes
            .into_iter()
            .map(|(t, r, s)| ((t, r).into(), s.raw))
            .collect();
        SharedShape::compound(shapes).into()
    }

    /// Initialize a new collider with a ball shape defined by its radius.
    pub fn ball(radius: Real) -> Self {
        SharedShape::ball(radius).into()
    }

    /// Initialize a new collider build with a half-space shape defined by the outward normal
    /// of its planar boundary.
    pub fn halfspace(outward_normal: Vect) -> Option<Self> {
        use rapier::na::Unit;
        let normal = Vector::from(outward_normal);
        Unit::try_new(normal, 1.0e-6).map(|n| SharedShape::halfspace(n).into())
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
        SharedShape::capsule(start.into(), end.into(), radius).into()
    }

    /// Initialize a new collider with a capsule shape aligned with the `x` axis.
    pub fn capsule_x(half_height: Real, radius: Real) -> Self {
        let p = Point::from(Vector::x() * half_height);
        SharedShape::capsule(-p, p, radius).into()
    }

    /// Initialize a new collider with a capsule shape aligned with the `y` axis.
    pub fn capsule_y(half_height: Real, radius: Real) -> Self {
        let p = Point::from(Vector::y() * half_height);
        SharedShape::capsule(-p, p, radius).into()
    }

    /// Initialize a new collider with a capsule shape aligned with the `z` axis.
    #[cfg(feature = "dim3")]
    pub fn capsule_z(half_height: Real, radius: Real) -> Self {
        let p = Point::from(Vector::z() * half_height);
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
        SharedShape::segment(a.into(), b.into()).into()
    }

    /// Initializes a collider with a triangle shape.
    pub fn triangle(a: Vect, b: Vect, c: Vect) -> Self {
        SharedShape::triangle(a.into(), b.into(), c.into()).into()
    }

    /// Initializes a collider with a triangle shape with round corners.
    pub fn round_triangle(a: Vect, b: Vect, c: Vect, border_radius: Real) -> Self {
        SharedShape::round_triangle(a.into(), b.into(), c.into(), border_radius).into()
    }

    /// Initializes a collider with a polyline shape defined by its vertex and index buffers.
    pub fn polyline(vertices: Vec<Vect>, indices: Option<Vec<[u32; 2]>>) -> Self {
        let vertices = vertices.into_iter().map(|v| v.into()).collect();
        SharedShape::polyline(vertices, indices).into()
    }

    /// Initializes a collider with a triangle mesh shape defined by its vertex and index buffers.
    pub fn trimesh(
        vertices: Vec<Vect>,
        indices: Vec<[u32; 3]>,
    ) -> Result<Self, crate::rapier::prelude::TriMeshBuilderError> {
        let vertices = vertices.into_iter().map(|v| v.into()).collect();
        Ok(SharedShape::trimesh(vertices, indices)?.into())
    }

    /// Initializes a collider with a triangle mesh shape defined by its vertex and index buffers, and flags
    /// controlling its pre-processing.
    pub fn trimesh_with_flags(
        vertices: Vec<Vect>,
        indices: Vec<[u32; 3]>,
        flags: TriMeshFlags,
    ) -> Result<Self, crate::rapier::prelude::TriMeshBuilderError> {
        let vertices = vertices.into_iter().map(|v| v.into()).collect();
        Ok(SharedShape::trimesh_with_flags(vertices, indices, flags)?.into())
    }

    /// Initializes a collider with a Bevy Mesh.
    ///
    /// Returns `None` if the index buffer or vertex buffer of the mesh are in an incompatible format.
    #[cfg(all(feature = "dim3", feature = "async-collider"))]
    pub fn from_bevy_mesh(mesh: &Mesh, collider_shape: &ComputedColliderShape) -> Option<Self> {
        let (vtx, idx) = extract_mesh_vertices_indices(mesh)?;

        match collider_shape {
            ComputedColliderShape::TriMesh(flags) => Some(
                SharedShape::trimesh_with_flags(vtx, idx, *flags)
                    .ok()?
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
    pub fn as_typed_shape(&self) -> ColliderView {
        self.raw.as_typed_shape().into()
    }

    /// Takes a strongly typed reference of the unscaled version of this collider.
    pub fn as_unscaled_typed_shape(&self) -> ColliderView {
        self.unscaled.as_typed_shape().into()
    }

    /// Downcast this collider to a ball, if it is one.
    pub fn as_ball(&self) -> Option<BallView> {
        self.raw.as_ball().map(|s| BallView { raw: s })
    }

    /// Downcast this collider to a cuboid, if it is one.
    pub fn as_cuboid(&self) -> Option<CuboidView> {
        self.raw.as_cuboid().map(|s| CuboidView { raw: s })
    }

    /// Downcast this collider to a capsule, if it is one.
    pub fn as_capsule(&self) -> Option<CapsuleView> {
        self.raw.as_capsule().map(|s| CapsuleView { raw: s })
    }

    /// Downcast this collider to a segment, if it is one.
    pub fn as_segment(&self) -> Option<SegmentView> {
        self.raw.as_segment().map(|s| SegmentView { raw: s })
    }

    /// Downcast this collider to a triangle, if it is one.
    pub fn as_triangle(&self) -> Option<TriangleView> {
        self.raw.as_triangle().map(|s| TriangleView { raw: s })
    }

    /// Downcast this collider to a triangle mesh, if it is one.
    pub fn as_trimesh(&self) -> Option<TriMeshView> {
        self.raw.as_trimesh().map(|s| TriMeshView { raw: s })
    }

    /// Downcast this collider to a polyline, if it is one.
    pub fn as_polyline(&self) -> Option<PolylineView> {
        self.raw.as_polyline().map(|s| PolylineView { raw: s })
    }

    /// Downcast this collider to a half-space, if it is one.
    pub fn as_halfspace(&self) -> Option<HalfSpaceView> {
        self.raw.as_halfspace().map(|s| HalfSpaceView { raw: s })
    }

    /// Downcast this collider to a heightfield, if it is one.
    pub fn as_heightfield(&self) -> Option<HeightFieldView> {
        self.raw
            .as_heightfield()
            .map(|s| HeightFieldView { raw: s })
    }

    /// Downcast this collider to a compound shape, if it is one.
    pub fn as_compound(&self) -> Option<CompoundView> {
        self.raw.as_compound().map(|s| CompoundView { raw: s })
    }

    /// Downcast this collider to a convex polygon, if it is one.
    #[cfg(feature = "dim2")]
    pub fn as_convex_polygon(&self) -> Option<ConvexPolygonView> {
        self.raw
            .as_convex_polygon()
            .map(|s| ConvexPolygonView { raw: s })
    }

    /// Downcast this collider to a convex polyhedron, if it is one.
    #[cfg(feature = "dim3")]
    pub fn as_convex_polyhedron(&self) -> Option<ConvexPolyhedronView> {
        self.raw
            .as_convex_polyhedron()
            .map(|s| ConvexPolyhedronView { raw: s })
    }

    /// Downcast this collider to a cylinder, if it is one.
    #[cfg(feature = "dim3")]
    pub fn as_cylinder(&self) -> Option<CylinderView> {
        self.raw.as_cylinder().map(|s| CylinderView { raw: s })
    }

    /// Downcast this collider to a cone, if it is one.
    #[cfg(feature = "dim3")]
    pub fn as_cone(&self) -> Option<ConeView> {
        self.raw.as_cone().map(|s| ConeView { raw: s })
    }

    /// Downcast this collider to a mutable ball, if it is one.
    pub fn as_ball_mut(&mut self) -> Option<BallViewMut> {
        self.raw
            .make_mut()
            .as_ball_mut()
            .map(|s| BallViewMut { raw: s })
    }

    /// Downcast this collider to a mutable cuboid, if it is one.
    pub fn as_cuboid_mut(&mut self) -> Option<CuboidViewMut> {
        self.raw
            .make_mut()
            .as_cuboid_mut()
            .map(|s| CuboidViewMut { raw: s })
    }

    /// Downcast this collider to a mutable capsule, if it is one.
    pub fn as_capsule_mut(&mut self) -> Option<CapsuleViewMut> {
        self.raw
            .make_mut()
            .as_capsule_mut()
            .map(|s| CapsuleViewMut { raw: s })
    }

    /// Downcast this collider to a mutable segment, if it is one.
    pub fn as_segment_mut(&mut self) -> Option<SegmentViewMut> {
        self.raw
            .make_mut()
            .as_segment_mut()
            .map(|s| SegmentViewMut { raw: s })
    }

    /// Downcast this collider to a mutable triangle, if it is one.
    pub fn as_triangle_mut(&mut self) -> Option<TriangleViewMut> {
        self.raw
            .make_mut()
            .as_triangle_mut()
            .map(|s| TriangleViewMut { raw: s })
    }

    /// Downcast this collider to a mutable triangle mesh, if it is one.
    pub fn as_trimesh_mut(&mut self) -> Option<TriMeshViewMut> {
        self.raw
            .make_mut()
            .as_trimesh_mut()
            .map(|s| TriMeshViewMut { raw: s })
    }

    /// Downcast this collider to a mutable polyline, if it is one.
    pub fn as_polyline_mut(&mut self) -> Option<PolylineViewMut> {
        self.raw
            .make_mut()
            .as_polyline_mut()
            .map(|s| PolylineViewMut { raw: s })
    }

    /// Downcast this collider to a mutable half-space, if it is one.
    pub fn as_halfspace_mut(&mut self) -> Option<HalfSpaceViewMut> {
        self.raw
            .make_mut()
            .as_halfspace_mut()
            .map(|s| HalfSpaceViewMut { raw: s })
    }

    /// Downcast this collider to a mutable heightfield, if it is one.
    pub fn as_heightfield_mut(&mut self) -> Option<HeightFieldViewMut> {
        self.raw
            .make_mut()
            .as_heightfield_mut()
            .map(|s| HeightFieldViewMut { raw: s })
    }

    // /// Downcast this collider to a mutable compound shape, if it is one.
    // pub fn as_compound_mut(&mut self) -> Option<CompoundViewMut> {
    //     self.raw.make_mut()
    //         .as_compound_mut()
    //         .map(|s| CompoundViewMut { raw: s })
    // }

    // /// Downcast this collider to a mutable convex polygon, if it is one.
    // #[cfg(feature = "dim2")]
    // pub fn as_convex_polygon_mut(&mut self) -> Option<ConvexPolygonViewMut> {
    //     self.raw.make_mut()
    //         .as_convex_polygon_mut()
    //         .map(|s| ConvexPolygonViewMut { raw: s })
    // }

    // /// Downcast this collider to a mutable convex polyhedron, if it is one.
    // #[cfg(feature = "dim3")]
    // pub fn as_convex_polyhedron_mut(&mut self) -> Option<ConvexPolyhedronViewMut> {
    //     self.raw.make_mut()
    //         .as_convex_polyhedron_mut()
    //         .map(|s| ConvexPolyhedronViewMut { raw: s })
    // }

    /// Downcast this collider to a mutable cylinder, if it is one.
    #[cfg(feature = "dim3")]
    pub fn as_cylinder_mut(&mut self) -> Option<CylinderViewMut> {
        self.raw
            .make_mut()
            .as_cylinder_mut()
            .map(|s| CylinderViewMut { raw: s })
    }

    /// Downcast this collider to a mutable cone, if it is one.
    #[cfg(feature = "dim3")]
    pub fn as_cone_mut(&mut self) -> Option<ConeViewMut> {
        self.raw
            .make_mut()
            .as_cone_mut()
            .map(|s| ConeViewMut { raw: s })
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

        if let Some(scaled) = self
            .as_unscaled_typed_shape()
            .raw_scale_by(scale, num_subdivisions)
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

    /// Projects a point on `self` transformed by `m`, unless the projection lies further than the given max distance.
    pub fn project_point_with_max_dist(
        &self,
        translation: Vect,
        rotation: Rot,
        point: Vect,
        solid: bool,
        max_dist: Real,
    ) -> Option<PointProjection> {
        let pos = (translation, rotation).into();
        self.raw
            .project_point_with_max_dist(&pos, &point.into(), solid, max_dist)
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

    /// Projects a point on `self` transformed by `m`.
    pub fn project_point(
        &self,
        translation: Vect,
        rotation: Rot,
        point: Vect,
        solid: bool,
    ) -> PointProjection {
        let pos = (translation, rotation).into();
        self.raw.project_point(&pos, &point.into(), solid).into()
    }

    /// Computes the minimal distance between a point and `self` transformed by `m`.
    #[inline]
    pub fn distance_to_point(
        &self,
        translation: Vect,
        rotation: Rot,
        point: Vect,
        solid: bool,
    ) -> Real {
        let pos = (translation, rotation).into();
        self.raw.distance_to_point(&pos, &point.into(), solid)
    }

    /// Projects a point on the boundary of `self` transformed by `m` and returns the id of the
    /// feature the point was projected on.
    pub fn project_point_and_get_feature(
        &self,
        translation: Vect,
        rotation: Rot,
        point: Vect,
    ) -> (PointProjection, FeatureId) {
        let pos = (translation, rotation).into();
        let (proj, feat) = self.raw.project_point_and_get_feature(&pos, &point.into());
        (proj.into(), feat)
    }

    /// Tests if the given point is inside of `self` transformed by `m`.
    pub fn contains_point(&self, translation: Vect, rotation: Rot, point: Vect) -> bool {
        let pos = (translation, rotation).into();
        self.raw.contains_point(&pos, &point.into())
    }

    /// Computes the time of impact between this transform shape and a ray.
    pub fn cast_local_ray(
        &self,
        ray_origin: Vect,
        ray_dir: Vect,
        max_time_of_impact: Real,
        solid: bool,
    ) -> Option<Real> {
        let ray = Ray::new(ray_origin.into(), ray_dir.into());
        self.raw.cast_local_ray(&ray, max_time_of_impact, solid)
    }

    /// Computes the time of impact, and normal between this transformed shape and a ray.
    pub fn cast_local_ray_and_get_normal(
        &self,
        ray_origin: Vect,
        ray_dir: Vect,
        max_time_of_impact: Real,
        solid: bool,
    ) -> Option<RayIntersection> {
        let ray = Ray::new(ray_origin.into(), ray_dir.into());
        self.raw
            .cast_local_ray_and_get_normal(&ray, max_time_of_impact, solid)
            .map(|inter| RayIntersection::from_rapier(inter, ray_origin, ray_dir))
    }

    /// Tests whether a ray intersects this transformed shape.
    pub fn intersects_local_ray(
        &self,
        ray_origin: Vect,
        ray_dir: Vect,
        max_time_of_impact: Real,
    ) -> bool {
        let ray = Ray::new(ray_origin.into(), ray_dir.into());
        self.raw.intersects_local_ray(&ray, max_time_of_impact)
    }

    /// Computes the time of impact between this transform shape and a ray.
    pub fn cast_ray(
        &self,
        translation: Vect,
        rotation: Rot,
        ray_origin: Vect,
        ray_dir: Vect,
        max_time_of_impact: Real,
        solid: bool,
    ) -> Option<Real> {
        let pos = (translation, rotation).into();
        let ray = Ray::new(ray_origin.into(), ray_dir.into());
        self.raw.cast_ray(&pos, &ray, max_time_of_impact, solid)
    }

    /// Computes the time of impact, and normal between this transformed shape and a ray.
    pub fn cast_ray_and_get_normal(
        &self,
        translation: Vect,
        rotation: Rot,
        ray_origin: Vect,
        ray_dir: Vect,
        max_time_of_impact: Real,
        solid: bool,
    ) -> Option<RayIntersection> {
        let pos = (translation, rotation).into();
        let ray = Ray::new(ray_origin.into(), ray_dir.into());
        self.raw
            .cast_ray_and_get_normal(&pos, &ray, max_time_of_impact, solid)
            .map(|inter| RayIntersection::from_rapier(inter, ray_origin, ray_dir))
    }

    /// Tests whether a ray intersects this transformed shape.
    pub fn intersects_ray(
        &self,
        translation: Vect,
        rotation: Rot,
        ray_origin: Vect,
        ray_dir: Vect,
        max_time_of_impact: Real,
    ) -> bool {
        let pos = (translation, rotation).into();
        let ray = Ray::new(ray_origin.into(), ray_dir.into());
        self.raw.intersects_ray(&pos, &ray, max_time_of_impact)
    }
}

impl Default for Collider {
    fn default() -> Self {
        Self::ball(0.5)
    }
}

#[cfg(all(feature = "dim3", feature = "async-collider"))]
#[allow(clippy::type_complexity)]
fn extract_mesh_vertices_indices(mesh: &Mesh) -> Option<(Vec<na::Point3<Real>>, Vec<[u32; 3]>)> {
    use rapier::na::point;

    let vertices = mesh.attribute(Mesh::ATTRIBUTE_POSITION)?;
    let indices = mesh.indices()?;

    let vtx: Vec<_> = match vertices {
        VertexAttributeValues::Float32(vtx) => Some(
            vtx.chunks(3)
                .map(|v| point![v[0] as Real, v[1] as Real, v[2] as Real])
                .collect(),
        ),
        VertexAttributeValues::Float32x3(vtx) => Some(
            vtx.iter()
                .map(|v| point![v[0] as Real, v[1] as Real, v[2] as Real])
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
