#pragma once

#include <algorithm>

#include "geom/primitives/point.hpp"

namespace geom {

struct Segment3d {
    Vec3 a{Vec3::Zero()};
    Vec3 b{Vec3::UnitX()};

    Vec3 project_point(const Vec3& p) const {
        Vec3 ab = b - a;
        Scalar t = ab.dot(p - a) / ab.squaredNorm();
        t = std::clamp(t, Scalar(0), Scalar(1));
        return a + t * ab;
    }

    Scalar distance(const Vec3& p) const {
        return (p - project_point(p)).norm();
    }

    Scalar length() const { return (b - a).norm(); }
};

} // namespace geom

