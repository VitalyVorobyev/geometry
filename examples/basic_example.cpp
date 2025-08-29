#include <geom.hpp>
#include <iostream>

using namespace geom;

int main() {
    std::cout << "=== Geometry Library Example ===\n\n";

    // Create a ray pointing down the Z-axis
    Ray3d ray;
    ray.origin = Ray3d::Vec3(0, 0, -2);
    ray.dir = Ray3d::Vec3(0, 0, 1);

    std::cout << "Ray origin: (" << ray.origin.transpose() << ")\n";
    std::cout << "Ray direction: (" << ray.dir.transpose() << ")\n\n";

    // Create a triangle in the XY plane
    Triangle3d triangle;
    triangle.a = Triangle3d::Vec3(-1, -1, 0);
    triangle.b = Triangle3d::Vec3( 1, -1, 0);
    triangle.c = Triangle3d::Vec3( 0,  1, 0);

    std::cout << "Triangle vertices:\n";
    std::cout << "  A: (" << triangle.a.transpose() << ")\n";
    std::cout << "  B: (" << triangle.b.transpose() << ")\n";
    std::cout << "  C: (" << triangle.c.transpose() << ")\n\n";

    // Calculate triangle properties
    std::cout << "Triangle area: " << triangle.area() << "\n";
    std::cout << "Triangle centroid: (" << triangle.centroid().transpose() << ")\n";
    std::cout << "Triangle normal: (" << triangle.normal().transpose() << ")\n\n";

    // Test intersection
    auto hit = intersect(ray, triangle);
    if (hit.hit) {
        std::cout << "✅ Ray intersects triangle!\n";
        std::cout << "  Distance (t): " << hit.t << "\n";
        std::cout << "  Barycentric coordinates: u=" << hit.u << ", v=" << hit.v << ", w=" << (1 - hit.u - hit.v) << "\n";

        // Calculate intersection point
        auto intersection_point = ray.point_at(hit.t);
        std::cout << "  Intersection point: (" << intersection_point.transpose() << ")\n";

        // Verify using barycentric coordinates
        auto barycentric_point = triangle.barycentric_to_cartesian(hit.u, hit.v);
        std::cout << "  Barycentric point: (" << barycentric_point.transpose() << ")\n";
    } else {
        std::cout << "❌ Ray does not intersect triangle\n";
    }

    // Test a miss case
    std::cout << "\n--- Testing miss case ---\n";
    Ray3d miss_ray = Ray3d::create_from_points(
        Ray3d::Vec3(5, 5, -1),
        Ray3d::Vec3(5, 5, 1)
    );

    auto miss_hit = intersect(miss_ray, triangle);
    if (!miss_hit.hit) {
        std::cout << "✅ Ray correctly misses triangle\n";
    } else {
        std::cout << "❌ Unexpected intersection\n";
    }

    // Demonstrate additional functionality
    std::cout << "\n--- Additional Features ---\n";

    // Create an equilateral triangle
    auto eq_triangle = Triangle3d::create_equilateral(Triangle3d::Vec3::Zero(), 3.0);
    std::cout << "Equilateral triangle area: " << eq_triangle.area() << "\n";
    std::cout << "Expected area: " << 3.0 * 3.0 * std::sqrt(3.0) / 4.0 << "\n";

    std::cout << "\n=== Example completed ===\n";
    return 0;
}
