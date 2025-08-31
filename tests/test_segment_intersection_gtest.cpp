#include <gtest/gtest.h>

#include "geom/algorithms/intersect.hpp"

using namespace geom;

TEST(SegmentSegment, BasicIntersection) {
    Vec2 a0(0, 0), a1(1, 0);
    Vec2 b0(0.5, -1), b1(0.5, 1);
    auto hit = intersect_segments(a0, a1, b0, b1);
    ASSERT_TRUE(hit.hit);
    EXPECT_NEAR(hit.point.x(), 0.5, 1e-6);
    EXPECT_NEAR(hit.point.y(), 0.0, 1e-6);
}

TEST(SegmentSegment, ParallelNoIntersection) {
    Vec2 a0(0, 0), a1(1, 0);
    Vec2 b0(0, 1), b1(1, 1);
    auto hit = intersect_segments(a0, a1, b0, b1);
    EXPECT_FALSE(hit.hit);
}

TEST(SegmentSegment, NearlyParallelIntersection) {
    Vec2 a0(0, 0), a1(1, 1e-8);
    Vec2 b0(0, 1e-8), b1(1, 0);
    auto hit = intersect_segments(a0, a1, b0, b1);
    ASSERT_TRUE(hit.hit);
    EXPECT_NEAR(hit.point.x(), 0.5, 1e-6);
    EXPECT_NEAR(hit.point.y(), 0.5e-8, 1e-10);
}

TEST(SegmentSegment, CollinearSharingEndpoint) {
    // Test the original issue: segments a0=(0,0)–a1=(1,0) and b0=(1,0)–b1=(2,0)
    // These should intersect at point (1,0)
    Vec2 a0(0, 0), a1(1, 0);
    Vec2 b0(1, 0), b1(2, 0);
    auto hit = intersect_segments(a0, a1, b0, b1);
    ASSERT_TRUE(hit.hit);
    EXPECT_TRUE(hit.collinear);
    EXPECT_NEAR(hit.point.x(), 1.0, 1e-6);
    EXPECT_NEAR(hit.point.y(), 0.0, 1e-6);
}

TEST(SegmentSegment, CollinearOverlapping) {
    // Test overlapping collinear segments
    Vec2 a0(0, 0), a1(2, 0);
    Vec2 b0(1, 0), b1(3, 0);
    auto hit = intersect_segments(a0, a1, b0, b1);
    ASSERT_TRUE(hit.hit);
    EXPECT_TRUE(hit.collinear);
    // Should report intersection at the start of the overlap
    EXPECT_NEAR(hit.point.x(), 1.0, 1e-6);
    EXPECT_NEAR(hit.point.y(), 0.0, 1e-6);
}
