#pragma once

// std
#include <cmath>

// eigen
#include <Eigen/Core>

#include "geom/core/config.hpp"

namespace geom {

struct Triangle3d final {
    using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
    Vec3 a{Vec3::Zero()}, b{Vec3::UnitX()}, c{Vec3::UnitY()};

    // Calculate triangle normal vector
    Vec3 normal() const;

    // Calculate triangle area
    Scalar area() const;

    // Calculate triangle centroid
    Vec3 centroid() const;

    // Convert barycentric coordinates to cartesian
    Vec3 barycentric_to_cartesian(Scalar u, Scalar v, Scalar w) const;
    Vec3 barycentric_to_cartesian(Scalar u, Scalar v) const; // w = 1 - u - v

    // Check if barycentric coordinates are within triangle bounds
    bool contains_point_barycentric(Scalar u, Scalar v, Scalar tolerance = Scalar(1e-6)) const;

    // Create equilateral triangle
    static Triangle3d create_equilateral(const Vec3& center, Scalar side_length);
};

}  // namespace geom
