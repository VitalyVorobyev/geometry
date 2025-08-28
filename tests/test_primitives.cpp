#include <cassert>
#include <cmath>
#include <iostream>

#include "geom/primitives/ray.hpp"
#include "geom/primitives/triangle.hpp"

using namespace geom;

void test_ray_functionality() {
    std::cout << "Testing Ray3d functionality...\n";
    
    // Test basic construction
    Ray3d ray;
    ray.origin = Ray3d::Vec3(1, 2, 3);
    ray.dir = Ray3d::Vec3(0, 0, 1);
    
    // Test point_at
    [[maybe_unused]] auto point = ray.point_at(2.0);
    assert(std::abs(point.x() - 1.0) < 1e-6);
    assert(std::abs(point.y() - 2.0) < 1e-6);
    assert(std::abs(point.z() - 5.0) < 1e-6);
    
    // Test normalize_direction
    ray.dir = Ray3d::Vec3(3, 4, 0);
    ray.normalize_direction();
    assert(std::abs(ray.dir.norm() - 1.0) < 1e-6);
    
    // Test create_from_points
    Ray3d::Vec3 start(0, 0, 0);
    Ray3d::Vec3 end(1, 1, 1);
    [[maybe_unused]] auto ray_from_points = Ray3d::create_from_points(start, end);
    assert(ray_from_points.is_direction_normalized());
    
    std::cout << "Ray tests passed!\n";
}

void test_triangle_functionality() {
    std::cout << "Testing Triangle3d functionality...\n";
    
    // Create a simple triangle
    Triangle3d tri;
    tri.a = Triangle3d::Vec3(0, 0, 0);
    tri.b = Triangle3d::Vec3(1, 0, 0);
    tri.c = Triangle3d::Vec3(0, 1, 0);
    
    // Test area calculation
    [[maybe_unused]] Scalar area = tri.area();
    assert(std::abs(area - 0.5) < 1e-6);
    
    // Test centroid
    [[maybe_unused]] auto centroid = tri.centroid();
    assert(std::abs(centroid.x() - 1.0/3.0) < 1e-6);
    assert(std::abs(centroid.y() - 1.0/3.0) < 1e-6);
    assert(std::abs(centroid.z() - 0.0) < 1e-6);
    
    // Test barycentric coordinates
    [[maybe_unused]] auto point = tri.barycentric_to_cartesian(Scalar(0.5), Scalar(0.3));
    assert(tri.contains_point_barycentric(Scalar(0.5), Scalar(0.3)));
    assert(!tri.contains_point_barycentric(Scalar(-0.1), Scalar(0.5)));
    
    // Test equilateral triangle creation
    auto eq_tri = Triangle3d::create_equilateral(Triangle3d::Vec3::Zero(), Scalar(2.0));
    [[maybe_unused]] Scalar eq_area = eq_tri.area();
    [[maybe_unused]] Scalar expected_area = std::sqrt(Scalar(3.0)); // Area of equilateral triangle with side 2
    assert(std::abs(eq_area - expected_area) < 1e-5);
    
    std::cout << "Triangle tests passed!\n";
}

// Note: No main function - this will be called from the main test runner
