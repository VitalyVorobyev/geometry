#pragma once

// std
#include <limits>

#include "geom/primitives/ray.hpp"
#include "geom/primitives/triangle.hpp"

namespace geom {

struct RayTriHit final {
    bool hit{false};
    Scalar t{Scalar(0)};  // ray parameter along r.dir
    Scalar u{Scalar(0)};  // barycentric
    Scalar v{Scalar(0)};  // barycentric
};

/**
 * Möller–Trumbore ray-triangle intersection.
 * Returns hit when t >= 0 and barycentrics are inside the triangle.
 * Note: Minimal epsilon policy for now; customize as you expand.
 */
inline RayTriHit intersect(const Ray3d& r, const Triangle3d& tri) {
    using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
    const Scalar EPS = Scalar(1e-9);

    const Vec3 edge1 = tri.b - tri.a;
    const Vec3 edge2 = tri.c - tri.a;
    const Vec3 pvec  = r.dir.cross(edge2);
    const Scalar det = edge1.dot(pvec);

    if (det > -EPS && det < EPS) return {}; // parallel

    const Scalar invDet = Scalar(1) / det;
    const Vec3 tvec = r.origin - tri.a;
    const Scalar u = tvec.dot(pvec) * invDet;
    if (u < Scalar(0) - EPS || u > Scalar(1) + EPS) return {};

    const Vec3 qvec = tvec.cross(edge1);
    const Scalar v = r.dir.dot(qvec) * invDet;
    if (v < Scalar(0) - EPS || u + v > Scalar(1) + EPS) return {};

    const Scalar t = edge2.dot(qvec) * invDet;
    if (t < Scalar(0)) return {};

    return {true, t, u, v};
}

// Additional intersection functions implemented in intersect.cpp
RayTriHit intersect_moller_trumbore(const Ray3d& r, const Triangle3d& tri, Scalar epsilon = Scalar(1e-9));
bool ray_intersects_triangle(const Ray3d& r, const Triangle3d& tri, Scalar epsilon = Scalar(1e-9));
Scalar distance_ray_to_triangle(const Ray3d& r, const Triangle3d& tri);

}  // namespace geom
