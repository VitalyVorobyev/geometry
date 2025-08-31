#include <chrono>
#include <iostream>
#include <vector>

#include "geom/spatial/kdtree.hpp"
#include "geom/spatial/octree.hpp"
#include "geom/spatial/bvh.hpp"

using namespace geom;
using namespace geom::spatial;

int main() {
    std::vector<Vec3> pts;
    pts.reserve(1000);
    for (int i = 0; i < 1000; ++i) pts.push_back(Vec3::Random());
    PointCloud pc{pts};
    auto t0 = std::chrono::high_resolution_clock::now();
    KDTree kd(pc);
    auto t1 = std::chrono::high_resolution_clock::now();
    std::cout << "kd-tree build: " << std::chrono::duration<double, std::milli>(t1 - t0).count() << " ms\n";
    auto knn = kd.knn_search(Vec3::Zero(), 10);
    std::cout << "kNN result count: " << knn.size() << "\n";

    AABB3d bounds(Vec3(-1,-1,-1), Vec3(1,1,1));
    Octree oct(bounds);
    t0 = std::chrono::high_resolution_clock::now();
    oct.build(pts);
    t1 = std::chrono::high_resolution_clock::now();
    std::cout << "octree build: " << std::chrono::duration<double, std::milli>(t1 - t0).count() << " ms\n";

    Triangle3d tri;
    tri.a = Vec3(0,0,0); tri.b = Vec3(1,0,0); tri.c = Vec3(0,1,0);
    std::vector<Triangle3d> tris(1000, tri);
    t0 = std::chrono::high_resolution_clock::now();
    BVH bvh(tris);
    t1 = std::chrono::high_resolution_clock::now();
    std::cout << "bvh build: " << std::chrono::duration<double, std::milli>(t1 - t0).count() << " ms\n";
    return 0;
}

