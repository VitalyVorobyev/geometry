#pragma once

#include <Eigen/Geometry>

#include "geom/primitives/point.hpp"

namespace geom {

using Plane3d = Eigen::Hyperplane<Scalar, 3>;

inline Plane3d plane_from_point_normal(const Vec3& p, const Vec3& n) {
    Vec3 nn = n.normalized();
    return Plane3d(nn, -nn.dot(p));
}

inline Plane3d plane_from_points(const Vec3& a, const Vec3& b, const Vec3& c) {
    Vec3 n = (b - a).cross(c - a).normalized();
    return Plane3d(n, -n.dot(a));
}

} // namespace geom

