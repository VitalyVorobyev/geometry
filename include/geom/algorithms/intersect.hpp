#pragma once

// std
#include <limits>
#include <cmath>

#include "geom/primitives/ray.hpp"
#include "geom/primitives/triangle.hpp"
#include "geom/algorithms/results.hpp"
#include "geom/primitives/point.hpp"  // Vec2, Vec3

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

/**
 * Result of line/segment intersection
 */
struct LineIntersection final {
    bool hit{false};          // Whether there is an intersection
    bool collinear{false};    // Whether lines are collinear
    Vec2 point;              // Intersection point (valid when hit is true)
    Scalar t_a{Scalar(0)};    // Parameter along first line/segment
    Scalar t_b{Scalar(0)};    // Parameter along second line/segment
};

/**
 * Compute intersection between two infinite lines.
 * Lines are defined by start and end points, but extend infinitely.
 * Returns hit for all valid intersections, including collinear lines.
 */
inline LineIntersection intersect_lines(const Vec2& a0, const Vec2& a1,
                                        const Vec2& b0, const Vec2& b1,
                                        Scalar epsilon = Scalar(1e-9)) {
    const Vec2 dir_a = a1 - a0;
    const Vec2 dir_b = b1 - b0;

    const Scalar cross = dir_a[0] * dir_b[1] - dir_a[1] * dir_b[0];

    // Check if lines are parallel (or nearly parallel)
    if (std::abs(cross) < epsilon) {
        // Lines are collinear - check if they overlap
        const Vec2 r = a0 - b0;
        const Scalar dot1 = r.dot(dir_b);
        // const Scalar dot2 = dir_a.dot(dir_b);

        // If lines are collinear, return a valid hit with collinear flag
        return {true, true, a0, Scalar(0), dot1 / dir_b.squaredNorm()};
    }

    // Lines intersect at a single point
    const Vec2 r = b0 - a0;
    const Scalar t_a = (r[0] * dir_b[1] - r[1] * dir_b[0]) / cross;
    const Scalar t_b = (r[0] * dir_a[1] - r[1] * dir_a[0]) / cross;

    return {true, false, a0 + t_a * dir_a, t_a, t_b};
}

/**
 * Compute intersection between two line segments.
 * Segments are defined by their endpoints.
 * Returns hit when segments intersect, including at endpoints.
 */
inline LineIntersection intersect_segments(const Vec2& a0, const Vec2& a1,
                                           const Vec2& b0, const Vec2& b1,
                                           Scalar epsilon = Scalar(1e-9)) {
    LineIntersection result = intersect_lines(a0, a1, b0, b1, epsilon);

    // No intersection at all
    if (!result.hit) {
        return result;
    }

    // For collinear segments, we need to check for overlap
    if (result.collinear) {
        // Project segments onto a common axis
        const Vec2 dir = a1 - a0;
        const Scalar len_sq = dir.squaredNorm();
        if (len_sq < epsilon * epsilon) {
            // Segment A is essentially a point - check if it's on segment B
            const Scalar t = (a0 - b0).dot(b1 - b0) / (b1 - b0).squaredNorm();
            if (t >= 0 - epsilon && t <= 1 + epsilon) {
                return {true, true, a0, 0, t};
            }
            return {false, true, Vec2(), 0, 0};
        }

        // Parameterize both segments relative to segment A
        const auto parameterize = [&](const Vec2& p) {
            return (p - a0).dot(dir) / len_sq;
        };

        const Scalar t_b0 = parameterize(b0);
        const Scalar t_b1 = parameterize(b1);

        // Check for overlap
        const Scalar t_min = std::min(t_b0, t_b1);
        const Scalar t_max = std::max(t_b0, t_b1);

        if (t_min <= 1 + epsilon && t_max >= 0 - epsilon) {
            // Segments overlap - return one of the intersection points
            const Scalar t = std::max(Scalar(0), std::min(Scalar(1), t_min));
            return {true, true, a0 + t * dir, t, (t == t_b0) ? Scalar(0) : Scalar(1)};
        }

        return {false, true, Vec2(), 0, 0};
    }

    // Regular intersection - check if it's within both segments
    if (result.t_a >= 0 - epsilon && result.t_a <= 1 + epsilon &&
        result.t_b >= 0 - epsilon && result.t_b <= 1 + epsilon) {
        return result;
    }

    // Intersection point outside segments
    return {false, false, Vec2(), result.t_a, result.t_b};
}

// Additional intersection functions implemented in intersect.cpp
RayTriHit intersect_moller_trumbore(const Ray3d& r, const Triangle3d& tri, Scalar epsilon = Scalar(1e-9));
bool ray_intersects_triangle(const Ray3d& r, const Triangle3d& tri, Scalar epsilon = Scalar(1e-9));
Scalar distance_ray_to_triangle(const Ray3d& r, const Triangle3d& tri);

// 2D line-line intersection. Lines defined by two points each.
// Returns parameters on each line and intersection point.
inline IntersectResult<2> intersect_lines(const Vec2& p0, const Vec2& p1,
                                          const Vec2& q0, const Vec2& q1,
                                          Scalar epsilon = Scalar(1e-9)) {
    const Vec2 r = p1 - p0;
    const Vec2 s = q1 - q0;
    const Scalar denom = r.x() * s.y() - r.y() * s.x();
    if (std::abs(denom) < epsilon) return {}; // parallel
    const Vec2 qp = q0 - p0;
    const Scalar t = (qp.x() * s.y() - qp.y() * s.x()) / denom; // on first
    const Scalar u = (qp.x() * r.y() - qp.y() * r.x()) / denom; // on second
    IntersectResult<2> res;
    res.hit = true;
    res.paramA = t;
    res.paramB = u;
    res.point = p0 + t * r;
    return res;
}

// 2D segment-segment intersection. Returns hit only when intersection is
// within both segments (inclusive).
inline IntersectResult<2> intersect_segments(const Vec2& a0, const Vec2& a1,
                                             const Vec2& b0, const Vec2& b1,
                                             Scalar epsilon = Scalar(1e-9)) {
    auto res = intersect_lines(a0, a1, b0, b1, epsilon);
    if (!res.hit) return res;
    if (res.paramA < Scalar(0) - epsilon || res.paramA > Scalar(1) + epsilon) {
        return {};
    }
    if (res.paramB < Scalar(0) - epsilon || res.paramB > Scalar(1) + epsilon) {
        return {};
    }
    return res;
}

}  // namespace geom
