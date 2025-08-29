#include "geom/primitives/triangle.hpp"

namespace geom {

Triangle3d::Vec3 Triangle3d::normal() const {
    Vec3 edge1 = b - a;
    Vec3 edge2 = c - a;
    return edge1.cross(edge2).normalized();
}

Scalar Triangle3d::area() const {
    Vec3 edge1 = b - a;
    Vec3 edge2 = c - a;
    return Scalar(0.5) * edge1.cross(edge2).norm();
}

Triangle3d::Vec3 Triangle3d::centroid() const {
    return (a + b + c) / Scalar(3);
}

Triangle3d::Vec3 Triangle3d::barycentric_to_cartesian(Scalar u, Scalar v, Scalar w) const {
    return u * a + v * b + w * c;
}

Triangle3d::Vec3 Triangle3d::barycentric_to_cartesian(Scalar u, Scalar v) const {
    Scalar w = Scalar(1) - u - v;
    return barycentric_to_cartesian(u, v, w);
}

bool Triangle3d::contains_point_barycentric(Scalar u, Scalar v, Scalar tolerance) const {
    Scalar w = Scalar(1) - u - v;
    return (u >= -tolerance) && (v >= -tolerance) && (w >= -tolerance) &&
           (u <= Scalar(1) + tolerance) && (v <= Scalar(1) + tolerance) && (w <= Scalar(1) + tolerance);
}

Triangle3d Triangle3d::create_equilateral(const Vec3& center, Scalar side_length) {
    Scalar height = side_length * std::sqrt(Scalar(3)) / Scalar(2);
    Scalar half_side = side_length / Scalar(2);

    Triangle3d tri;
    tri.a = center + Vec3(-half_side, -height / Scalar(3), Scalar(0));
    tri.b = center + Vec3(half_side, -height / Scalar(3), Scalar(0));
    tri.c = center + Vec3(Scalar(0), Scalar(2) * height / Scalar(3), Scalar(0));

    return tri;
}

} // namespace geom
