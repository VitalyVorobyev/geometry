#include <gtest/gtest.h>

#include "geom/spatial/kdtree.hpp"
#include "geom/spatial/octree.hpp"
#include "geom/spatial/bvh.hpp"

using namespace geom;
using namespace geom::spatial;

TEST(KDTreeTest, RadiusAndKNN) {
    PointCloud pc;
    pc.points = {Vec3(0,0,0), Vec3(1,0,0), Vec3(0,1,0)};
    KDTree tree(pc);
    auto res = tree.radius_search(Vec3(0,0,0), Scalar(0.5));
    EXPECT_EQ(res.size(), 1u);
    auto knn = tree.knn_search(Vec3(0.2,0.2,0), 2);
    EXPECT_EQ(knn.size(), 2u);
    EXPECT_EQ(knn[0], 0u);
}

TEST(OctreeTest, BroadPhase) {
    std::vector<Vec3> pts = {Vec3(0,0,0), Vec3(1,1,1), Vec3(-1,-1,-1)};
    AABB3d bounds(Vec3(-2,-2,-2), Vec3(2,2,2));
    Octree tree(bounds);
    tree.build(pts);
    AABB3d region(Vec3(-0.5,-0.5,-0.5), Vec3(0.5,0.5,0.5));
    auto ids = tree.query(region);
    EXPECT_EQ(ids.size(), 1u);
}

TEST(BVHTest, RayHit) {
    Triangle3d tri;
    tri.a = Vec3(0,0,0);
    tri.b = Vec3(1,0,0);
    tri.c = Vec3(0,1,0);
    std::vector<Triangle3d> tris = {tri};
    BVH bvh(tris);
    Ray3d ray;
    ray.origin = Vec3(0.25,0.25,-1);
    ray.dir = Vec3(0,0,1);
    Scalar t; std::size_t idx; 
    bool hit = bvh.intersect(ray, t, idx);
    EXPECT_TRUE(hit);
    EXPECT_EQ(idx, 0u);
}

