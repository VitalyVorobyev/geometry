#include <gtest/gtest.h>
#include "geom/primitives/ray.hpp"
#include "geom/primitives/triangle.hpp"

using namespace geom;

class RayTest : public ::testing::Test {
protected:
    void SetUp() override {
        ray.origin = Ray3d::Vec3(1, 2, 3);
        ray.dir = Ray3d::Vec3(0, 0, 1);
    }
    
    Ray3d ray;
};

TEST_F(RayTest, PointAt) {
    auto point = ray.point_at(2.0);
    EXPECT_NEAR(point.x(), 1.0, 1e-6);
    EXPECT_NEAR(point.y(), 2.0, 1e-6);
    EXPECT_NEAR(point.z(), 5.0, 1e-6);
}

TEST_F(RayTest, NormalizeDirection) {
    ray.dir = Ray3d::Vec3(3, 4, 0);
    ray.normalize_direction();
    EXPECT_NEAR(ray.dir.norm(), 1.0, 1e-6);
}

TEST_F(RayTest, IsDirectionNormalized) {
    ray.dir = Ray3d::Vec3(1, 0, 0);
    EXPECT_TRUE(ray.is_direction_normalized());
    
    ray.dir = Ray3d::Vec3(2, 0, 0);
    EXPECT_FALSE(ray.is_direction_normalized());
}

TEST_F(RayTest, CreateFromPoints) {
    Ray3d::Vec3 start(0, 0, 0);
    Ray3d::Vec3 end(1, 1, 1);
    auto ray_from_points = Ray3d::create_from_points(start, end);
    
    EXPECT_TRUE(ray_from_points.is_direction_normalized());
    EXPECT_EQ(ray_from_points.origin, start);
}

class TriangleTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Simple triangle in XY plane
        tri.a = Triangle3d::Vec3(0, 0, 0);
        tri.b = Triangle3d::Vec3(1, 0, 0);
        tri.c = Triangle3d::Vec3(0, 1, 0);
    }
    
    Triangle3d tri;
};

TEST_F(TriangleTest, Area) {
    Scalar area = tri.area();
    EXPECT_NEAR(area, 0.5, 1e-6);
}

TEST_F(TriangleTest, Normal) {
    auto normal = tri.normal();
    EXPECT_NEAR(normal.x(), 0.0, 1e-6);
    EXPECT_NEAR(normal.y(), 0.0, 1e-6);
    EXPECT_NEAR(normal.z(), 1.0, 1e-6);
}

TEST_F(TriangleTest, Centroid) {
    auto centroid = tri.centroid();
    EXPECT_NEAR(centroid.x(), 1.0/3.0, 1e-6);
    EXPECT_NEAR(centroid.y(), 1.0/3.0, 1e-6);
    EXPECT_NEAR(centroid.z(), 0.0, 1e-6);
}

TEST_F(TriangleTest, BarycentricCoordinates) {
    auto point = tri.barycentric_to_cartesian(Scalar(0.5), Scalar(0.3));
    // Test that the point is valid (not NaN or infinity)
    EXPECT_FALSE(point.hasNaN());
    EXPECT_TRUE(point.allFinite());
}

TEST_F(TriangleTest, EquilateralTriangle) {
    auto eq_tri = Triangle3d::create_equilateral(Triangle3d::Vec3::Zero(), Scalar(2.0));
    Scalar eq_area = eq_tri.area();
    Scalar expected_area = std::sqrt(Scalar(3.0)); // Area of equilateral triangle with side 2
    EXPECT_NEAR(eq_area, expected_area, 1e-5);
}
