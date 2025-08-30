#pragma once

#include <Eigen/Geometry>

#include "geom/primitives/point.hpp"

namespace geom {

using AABB3d = Eigen::AlignedBox<Scalar, 3>;

inline Vec3 project_point(const AABB3d& box, const Vec3& p) {
    return p.cwiseMax(box.min()).cwiseMin(box.max());
}

inline Scalar signed_distance(const AABB3d& box, const Vec3& p) {
    Vec3 c = box.center();
    Vec3 e = box.sizes() / Scalar(2);
    Vec3 q = (p - c).cwiseAbs() - e;
    Vec3 q_clamped = q.cwiseMax(Vec3::Zero());
    return q_clamped.norm() + std::min(q.maxCoeff(), Scalar(0));
}

} // namespace geom
