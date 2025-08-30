#include <gtest/gtest.h>
#include <random>

#include "geom/primitives/line.hpp"
#include "geom/primitives/segment.hpp"
#include "geom/primitives/ray.hpp"
#include "geom/primitives/plane.hpp"
#include "geom/primitives/triangle.hpp"
#include "geom/primitives/aabb.hpp"
#include "geom/primitives/obb.hpp"
#include "geom/core/predicates.hpp"

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

TEST_F(TriangleTest, CartesianToBarycentric) {
    auto coords = tri.cartesian_to_barycentric(tri.centroid());
    EXPECT_NEAR(coords[0], Scalar(1.0/3.0), 1e-6);
    EXPECT_NEAR(coords[1], Scalar(1.0/3.0), 1e-6);
    EXPECT_NEAR(coords[2], Scalar(1.0/3.0), 1e-6);
}

TEST_F(TriangleTest, BarycentricRoundTripRandom) {
    std::mt19937 gen(42);
    std::uniform_real_distribution<Scalar> dist(0.0, 1.0);
    for(int i=0;i<100;i++) {
        Scalar u = dist(gen);
        Scalar v = dist(gen);
        if(u + v > Scalar(1)) { u = Scalar(1) - u; v = Scalar(1) - v; }
        Scalar w = Scalar(1) - u - v;
        auto p = tri.barycentric_to_cartesian(u,v,w);
        auto coords = tri.cartesian_to_barycentric(p);
        EXPECT_NEAR(coords[0], u, 1e-5);
        EXPECT_NEAR(coords[1], v, 1e-5);
        EXPECT_NEAR(coords[2], w, 1e-5);
    }
}

TEST(LineTest, Projection) {
    Line3d line(Vec3::Zero(), Vec3::UnitX());
    Vec3 p(1,1,0);
    auto proj = line.projection(p);
    EXPECT_NEAR(proj.y(), 0.0, 1e-6);
}

TEST(LineTest, ProjectionRandom) {
    std::mt19937 gen(123);
    std::uniform_real_distribution<Scalar> dist(-1.0,1.0);
    for(int i=0;i<100;i++) {
        Vec3 a(dist(gen), dist(gen), dist(gen));
        Vec3 b(dist(gen), dist(gen), dist(gen));
        if((b-a).norm() < 1e-6) continue;
        Line3d line = line_from_points(a,b);
        Vec3 p(dist(gen), dist(gen), dist(gen));
        Vec3 proj = line.projection(p);
        Vec3 v = p - proj;
        EXPECT_NEAR(v.dot(line.direction()), 0.0, 1e-5);
    }
}

TEST(SegmentTest, Distance) {
    Segment3d seg;
    seg.a = Vec3(0,0,0);
    seg.b = Vec3(1,0,0);
    Vec3 p(0.5,1,0);
    EXPECT_NEAR(seg.distance(p), 1.0, 1e-6);
}

TEST(SegmentTest, Length) {
    Segment3d seg;
    seg.a = Vec3(0, 0, 0);
    seg.b = Vec3(3, 4, 0);

    // Length should be 5 (3-4-5 triangle)
    EXPECT_NEAR(seg.length(), 5.0, 1e-6);

    // Test with different segment
    seg.a = Vec3(-1, -1, -1);
    seg.b = Vec3(1, 1, 1);
    Scalar expected = std::sqrt(12.0);  // sqrt((2)^2 + (2)^2 + (2)^2)
    EXPECT_NEAR(seg.length(), expected, 1e-6);
}

TEST(PlaneTest, SignedDistance) {
    Plane3d plane = plane_from_point_normal(Vec3::Zero(), Vec3::UnitZ());
    Vec3 p(0,0,5);
    EXPECT_NEAR(plane.signedDistance(p), 5.0, 1e-6);
    auto proj = plane.projection(p);
    EXPECT_NEAR(proj.z(), 0.0, 1e-6);
}

TEST(PlaneTest, PlaneFromPoints) {
    // Create a plane from three points in the XY plane (z=0)
    Vec3 a(0, 0, 0);
    Vec3 b(1, 0, 0);
    Vec3 c(0, 1, 0);

    Plane3d plane = plane_from_points(a, b, c);

    // The normal should point in the Z direction
    Vec3 expected_normal(0, 0, 1);
    EXPECT_NEAR(plane.normal().dot(expected_normal), 1.0, 1e-6);

    // All three points should be on the plane (distance = 0)
    EXPECT_NEAR(std::abs(plane.signedDistance(a)), 0.0, 1e-6);
    EXPECT_NEAR(std::abs(plane.signedDistance(b)), 0.0, 1e-6);
    EXPECT_NEAR(std::abs(plane.signedDistance(c)), 0.0, 1e-6);

    // A point above the plane should have positive distance
    Vec3 above(0, 0, 1);
    EXPECT_GT(plane.signedDistance(above), 0.0);
}

TEST(AABBTest, ProjectionAndDistance) {
    AABB3d box(Vec3(-1,-1,-1), Vec3(1,1,1));
    Vec3 p(2,0,0);
    auto proj = project_point(box, p);
    EXPECT_NEAR(proj.x(), 1.0, 1e-6);
    EXPECT_NEAR(signed_distance(box, p), 1.0, 1e-6);
    EXPECT_TRUE(signed_distance(box, Vec3(0,0,0)) <= 0);
}

TEST(OBBTest, Projection) {
    OBB3d box; box.center = Vec3::Zero(); box.half_extents = Vec3(1,2,3);
    Vec3 p(3,0,0);
    auto proj = box.project_point(p);
    EXPECT_NEAR((proj - Vec3(1,0,0)).norm(), 0.0, 1e-6);
}

TEST(OBBTest, Contains) {
    OBB3d box;
    box.center = Vec3::Zero();
    box.half_extents = Vec3(1, 1, 1);

    // Point inside the box
    EXPECT_TRUE(box.contains(Vec3(0.5, 0.5, 0.5)));
    EXPECT_TRUE(box.contains(Vec3::Zero()));

    // Point outside the box
    EXPECT_FALSE(box.contains(Vec3(2, 0, 0)));
    EXPECT_FALSE(box.contains(Vec3(0, 2, 0)));
    EXPECT_FALSE(box.contains(Vec3(0, 0, 2)));

    // Point on the boundary
    EXPECT_TRUE(box.contains(Vec3(1, 0, 0)));
    EXPECT_TRUE(box.contains(Vec3(0, 1, 0)));
    EXPECT_TRUE(box.contains(Vec3(0, 0, 1)));
}

TEST(PredicateTest, Orientation2D) {
    Vec2 a(0,0), b(1,0), c(0,1);
    EXPECT_GT(orient2d(a,b,c), 0);
}

TEST(PredicateTest, Orientation3D) {
    Vec3 a(0,0,0), b(1,0,0), c(0,1,0), d(0,0,1), e(0,0,-1);
    EXPECT_LT(orient3d(a,b,c,d), 0);
    EXPECT_GT(orient3d(a,b,c,e), 0);
}
