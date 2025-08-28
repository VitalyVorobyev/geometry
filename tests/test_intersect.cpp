#include <cassert>
#include <cmath>
#include <iostream>

#include "geom/primitives/ray.hpp"
#include "geom/primitives/triangle.hpp"
#include "geom/algorithms/intersect.hpp"

using namespace geom;

void test_intersect_functionality() {
  std::cout << "Testing intersection algorithms...\n";
  
  Ray3d r;
  r.origin = Ray3d::Vec3(0, 0, -1);
  r.dir    = Ray3d::Vec3(0, 0, 1);

  Triangle3d tri;
  tri.a = Triangle3d::Vec3(-1, -1, 0);
  tri.b = Triangle3d::Vec3( 1, -1, 0);
  tri.c = Triangle3d::Vec3( 0,  1, 0);

  auto hit = intersect(r, tri);
  assert(hit.hit && "Ray should hit the triangle");
  assert(std::abs(hit.t - 1.0) < 1e-6);
  assert(hit.u >= -1e-6 && hit.v >= -1e-6 && hit.u + hit.v <= 1.0 + 1e-6);

  // Test ray_intersects_triangle function
  assert(ray_intersects_triangle(r, tri));
  
  // Test miss case
  Ray3d miss_ray;
  miss_ray.origin = Ray3d::Vec3(10, 10, -1);
  miss_ray.dir = Ray3d::Vec3(0, 0, 1);
  auto miss_hit = intersect(miss_ray, tri);
  assert(!miss_hit.hit && "Ray should miss the triangle");

  std::cout << "Intersection tests passed!\n";
}
