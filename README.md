# Geometry Library

A modular C++20 computational geometry library.

## Features

- **Modular Design**: Separate targets for core, primitives, and algorithms
- **Modern C++20**: Uses latest language features and best practices
- **Eigen Integration**: Built on the robust Eigen linear algebra library
- **Comprehensive Testing**: Unit tests for all modules with CI/CD
- **Cross-Platform**: Supports Linux, Windows, and macOS

## What's Included

### Core Module (`geom_core`)
- Configurable scalar types (float/double precision)
- Eigen integration and type aliases

### Primitives Module (`geom_primitives`)
- **Ray3d**: 3D ray with origin and direction
- **Triangle3d**: 3D triangle with utility methods

### Algorithms Module (`geom_algorithms`)
- **Ray-Triangle Intersection**: Möller–Trumbore algorithm
- **Distance Calculations**: Ray to triangle distance
- **Intersection Testing**: Fast boolean intersection checks

## Quick Start

### Installation

```bash
git clone https://github.com/VitalyVorobyev/geometry.git
cd geometry
mkdir build && cd build
cmake .. -DGEOM_BUILD_TESTS=ON
cmake --build .
```

### Usage

```cpp
#include <geom.hpp>

using namespace geom;

// Create a ray
Ray3d ray;
ray.origin = Ray3d::Vec3(0, 0, -1);
ray.dir = Ray3d::Vec3(0, 0, 1);

// Create a triangle
Triangle3d triangle;
triangle.a = Triangle3d::Vec3(-1, -1, 0);
triangle.b = Triangle3d::Vec3( 1, -1, 0);
triangle.c = Triangle3d::Vec3( 0,  1, 0);

// Test intersection
auto hit = intersect(ray, triangle);
if (hit.hit) {
    std::cout << "Intersection at t=" << hit.t << std::endl;
}
```

## Build Options

- `GEOM_USE_DOUBLE`: Use double precision (default: ON)
- `GEOM_BUILD_TESTS`: Build unit tests (default: ON)

## Testing

```bash
# Run all tests
ctest --output-on-failure

# Run specific test modules
ctest -R primitives_tests
ctest -R intersect_tests
```

## Project Structure

```
geometry/
├── include/geom/          # Public API headers
│   ├── geom.hpp          # Main convenience header
│   ├── core/             # Core configuration
│   ├── primitives/       # Geometric primitives
│   └── algorithms/       # Geometric algorithms
├── src/                  # Implementation files
│   ├── core/
│   ├── primitives/
│   └── algorithms/
├── tests/                # Comprehensive test suite
├── .github/workflows/    # CI/CD automation
└── cmake/               # CMake configuration
```

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) for development guidelines.

## License

See [LICENSE](LICENSE) file for details.
