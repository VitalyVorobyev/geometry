#pragma once

#include <limits>

#include "geom/primitives/point.hpp"

namespace geom {

/** Result of intersection queries. Stores parameters on each primitive
 * and the computed intersection point if any. */
template<int Dim>
struct IntersectResult {
    bool hit{false};
    Scalar paramA{Scalar(0)}; ///< Parameter on first primitive
    Scalar paramB{Scalar(0)}; ///< Parameter on second primitive
    Eigen::Matrix<Scalar, Dim, 1> point{Eigen::Matrix<Scalar, Dim, 1>::Zero()};
};

/** Result of distance queries. Contains closest points and distance. */
template<int Dim>
struct DistanceResult {
    Scalar distance{std::numeric_limits<Scalar>::infinity()};
    Eigen::Matrix<Scalar, Dim, 1> closestA{Eigen::Matrix<Scalar, Dim, 1>::Zero()};
    Eigen::Matrix<Scalar, Dim, 1> closestB{Eigen::Matrix<Scalar, Dim, 1>::Zero()};
};

} // namespace geom

