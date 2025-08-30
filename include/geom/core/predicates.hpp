#pragma once

#include "geom/primitives/point.hpp"

namespace geom {

/**
 * @brief Computes the 2D orientation determinant for three points.
 *
 * This function determines the orientation of the triplet of points (a, b, c) in 2D space.
 * It calculates the signed area of the triangle formed by these points, which can be used
 * to determine the relative orientation:
 * - If the result is positive, the points are in counter-clockwise order.
 * - If the result is negative, the points are in clockwise order.
 * - If the result is zero, the points are collinear.
 *
 * @param a The first point (Vec2).
 * @param b The second point (Vec2).
 * @param c The third point (Vec2).
 * @return Scalar The signed area of the triangle formed by the points.
 */
inline Scalar orient2d(const Vec2& a, const Vec2& b, const Vec2& c) {
    return (b.x() - a.x()) * (c.y() - a.y()) - (b.y() - a.y()) * (c.x() - a.x());
}

/**
 * @brief Computes the orientation of a point relative to a plane defined by three other points in 3D space.
 *
 * This function calculates the signed volume of the tetrahedron formed by the points `a`, `b`, `c`, and `d`.
 * The sign of the result indicates the orientation of the point `d` relative to the plane defined by `a`, `b`, and `c`:
 * - A positive value indicates that `d` is above the plane.
 * - A negative value indicates that `d` is below the plane.
 * - A zero value indicates that `d` lies on the plane.
 *
 * @param a The first point defining the plane.
 * @param b The second point defining the plane.
 * @param c The third point defining the plane.
 * @param d The point whose orientation relative to the plane is to be determined.
 * @return The signed scalar value representing the orientation of `d` relative to the plane.
 */
inline Scalar orient3d(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d) {
    Vec3 ad = a - d;
    Vec3 bd = b - d;
    Vec3 cd = c - d;
    return ad.dot(bd.cross(cd));
}

} // namespace geom
