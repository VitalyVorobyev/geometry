#include "geom/primitives/ray.hpp"
#include <cmath>

namespace geom {

Ray3d::Vec3 Ray3d::point_at(Scalar t) const {
    return origin + t * dir;
}

void Ray3d::normalize_direction() {
    Scalar norm = dir.norm();
    if (norm > std::numeric_limits<Scalar>::epsilon()) {
        dir /= norm;
    }
}

bool Ray3d::is_direction_normalized(Scalar tolerance) const {
    return std::abs(dir.norm() - Scalar(1)) < tolerance;
}

Ray3d Ray3d::create_from_points(const Vec3& start, const Vec3& end) {
    Ray3d ray;
    ray.origin = start;
    ray.dir = (end - start).normalized();
    return ray;
}

} // namespace geom
