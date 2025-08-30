#pragma once

#include <Eigen/Geometry>

#include "geom/primitives/point.hpp"

namespace geom {

using Line3d = Eigen::ParametrizedLine<Scalar, 3>;

inline Line3d line_from_points(const Vec3& a, const Vec3& b) {
    return Line3d::Through(a, b);
}

} // namespace geom

