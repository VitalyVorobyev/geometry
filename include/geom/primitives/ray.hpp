#pragma once

#include <Eigen/Core>
#include <limits>

#include "../core/config.hpp"

namespace geom {

struct Ray3d {
    using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
    Vec3 origin{Vec3::Zero()};
    Vec3 dir{Vec3::UnitX()}; // should be normalized
    
    // Get point along ray at parameter t
    Vec3 point_at(Scalar t) const;
    
    // Normalize the direction vector
    void normalize_direction();
    
    // Check if direction is normalized within tolerance
    bool is_direction_normalized(Scalar tolerance = Scalar(1e-6)) const;
    
    // Create ray from two points
    static Ray3d create_from_points(const Vec3& start, const Vec3& end);
};

}  // namespace geom
