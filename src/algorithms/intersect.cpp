#include "geom/algorithms/intersect.hpp"
#include <cmath>

namespace geom {

RayTriHit intersect_moller_trumbore(const Ray3d& r, const Triangle3d& tri, Scalar epsilon) {
    using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
    
    const Vec3 edge1 = tri.b - tri.a;
    const Vec3 edge2 = tri.c - tri.a;
    const Vec3 pvec = r.dir.cross(edge2);
    const Scalar det = edge1.dot(pvec);

    // Check if ray is parallel to triangle
    if (det > -epsilon && det < epsilon) {
        return RayTriHit{}; // No intersection
    }

    const Scalar invDet = Scalar(1) / det;
    const Vec3 tvec = r.origin - tri.a;
    const Scalar u = tvec.dot(pvec) * invDet;
    
    // Check if intersection point is outside triangle (u < 0 or u > 1)
    if (u < Scalar(0) - epsilon || u > Scalar(1) + epsilon) {
        return RayTriHit{};
    }

    const Vec3 qvec = tvec.cross(edge1);
    const Scalar v = r.dir.dot(qvec) * invDet;
    
    // Check if intersection point is outside triangle (v < 0 or u + v > 1)
    if (v < Scalar(0) - epsilon || u + v > Scalar(1) + epsilon) {
        return RayTriHit{};
    }

    const Scalar t = edge2.dot(qvec) * invDet;
    
    // Check if intersection is behind ray origin
    if (t < Scalar(0)) {
        return RayTriHit{};
    }

    return RayTriHit{true, t, u, v};
}

bool ray_intersects_triangle(const Ray3d& r, const Triangle3d& tri, Scalar epsilon) {
    return intersect_moller_trumbore(r, tri, epsilon).hit;
}

Scalar distance_ray_to_triangle(const Ray3d& r, const Triangle3d& tri) {
    auto hit = intersect(r, tri);
    if (hit.hit) {
        return hit.t * r.dir.norm();
    }
    return std::numeric_limits<Scalar>::infinity();
}

} // namespace geom
