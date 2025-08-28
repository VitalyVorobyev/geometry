#include <gtest/gtest.h>
#include "geom/algorithms/intersect.hpp"

using namespace geom;

class IntersectionTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Ray pointing down Z-axis
        ray.origin = Ray3d::Vec3(0, 0, -1);
        ray.dir = Ray3d::Vec3(0, 0, 1);
        
        // Triangle in XY plane
        triangle.a = Triangle3d::Vec3(-1, -1, 0);
        triangle.b = Triangle3d::Vec3( 1, -1, 0);
        triangle.c = Triangle3d::Vec3( 0,  1, 0);
    }
    
    Ray3d ray;
    Triangle3d triangle;
};

TEST_F(IntersectionTest, BasicIntersection) {
    auto hit = intersect(ray, triangle);
    
    EXPECT_TRUE(hit.hit) << "Ray should hit the triangle";
    EXPECT_NEAR(hit.t, 1.0, 1e-6);
    EXPECT_GE(hit.u, -1e-6);
    EXPECT_GE(hit.v, -1e-6);
    EXPECT_LE(hit.u + hit.v, 1.0 + 1e-6);
}

TEST_F(IntersectionTest, RayIntersectsTriangle) {
    EXPECT_TRUE(ray_intersects_triangle(ray, triangle));
}

TEST_F(IntersectionTest, DistanceCalculation) {
    Scalar distance = distance_ray_to_triangle(ray, triangle);
    EXPECT_NEAR(distance, 1.0, 1e-6) << "Distance should be 1.0";
}

TEST_F(IntersectionTest, MissCase) {
    // Ray that misses the triangle
    Ray3d miss_ray;
    miss_ray.origin = Ray3d::Vec3(10, 10, -1);
    miss_ray.dir = Ray3d::Vec3(0, 0, 1);
    
    auto miss_hit = intersect(miss_ray, triangle);
    EXPECT_FALSE(miss_hit.hit) << "Ray should miss the triangle";
    
    // Distance should be infinity for miss
    Scalar miss_distance = distance_ray_to_triangle(miss_ray, triangle);
    EXPECT_TRUE(std::isinf(miss_distance)) << "Miss distance should be infinity";
}

TEST_F(IntersectionTest, ParallelRay) {
    // Ray parallel to triangle plane
    Ray3d parallel_ray;
    parallel_ray.origin = Ray3d::Vec3(0, 0, 1);
    parallel_ray.dir = Ray3d::Vec3(1, 0, 0);
    
    auto hit = intersect(parallel_ray, triangle);
    EXPECT_FALSE(hit.hit) << "Parallel ray should not intersect";
}

TEST_F(IntersectionTest, BackfacingRay) {
    // Ray pointing away from triangle
    Ray3d backward_ray;
    backward_ray.origin = Ray3d::Vec3(0, 0, -1);
    backward_ray.dir = Ray3d::Vec3(0, 0, -1);
    
    auto hit = intersect(backward_ray, triangle);
    EXPECT_FALSE(hit.hit) << "Backward ray should not intersect";
}

TEST_F(IntersectionTest, MollerTrumboreWithCustomEpsilon) {
    // Test with custom epsilon
    auto hit = intersect_moller_trumbore(ray, triangle, Scalar(1e-12));
    EXPECT_TRUE(hit.hit);
    EXPECT_NEAR(hit.t, 1.0, 1e-6);
}
