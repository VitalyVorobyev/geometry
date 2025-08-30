#pragma once

#include "geom/primitives/point.hpp"

namespace geom {

inline Scalar orient2d(const Vec2& a, const Vec2& b, const Vec2& c) {
    return (b.x() - a.x()) * (c.y() - a.y()) - (b.y() - a.y()) * (c.x() - a.x());
}

inline Scalar orient3d(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d) {
    Vec3 ad = a - d;
    Vec3 bd = b - d;
    Vec3 cd = c - d;
    return ad.dot(bd.cross(cd));
}

} // namespace geom

