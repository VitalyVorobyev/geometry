#pragma once

#include <Eigen/Core>

#include "geom/core/config.hpp"

namespace geom {

using Vec2 = Eigen::Matrix<Scalar, 2, 1>;
using Vec3 = Eigen::Matrix<Scalar, 3, 1>;

using Point2d = Vec2;
using Point3d = Vec3;

} // namespace geom

