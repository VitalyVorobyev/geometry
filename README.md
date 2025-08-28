# Geometry Library

A modular C++20 computational geometry library with comprehensive testing.

## Features

- **Modular CMake Architecture**: Dedicated CMake files for each module
- **Modern C++20**: Uses latest language features and best practices  
- **Eigen Integration**: Built on the robust Eigen linear algebra library
- **Google Test Framework**: Professional testing with comprehensive coverage
- **Cross-Platform**: Supports Linux, Windows, and macOS
- **Continuous Integration**: Automated testing, static analysis, and code coverage

## What's Included

### Core Module (`geom_core`)
- Configurable scalar types (float/double precision)
- Eigen integration and type aliases
- Project-wide configuration header

### Primitives Module (`geom_primitives`) 
- **Ray3d**: 3D ray with origin and direction
- **Triangle3d**: 3D triangle with utility methods

### Algorithms Module (`geom_algorithms`)
- **Ray-Triangle Intersection**: Möller–Trumbore algorithm
- **Distance Calculations**: Ray to triangle distance
- **Intersection Testing**: Fast boolean intersection checks

## Quick Start

### Prerequisites

- CMake 3.22 or higher
- C++20 compatible compiler
- Eigen3 library (auto-downloaded by CMake)
- Google Test (auto-downloaded by CMake)

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
#include "geom/primitives/ray.hpp"
#include "geom/primitives/triangle.hpp"
#include "geom/algorithms/intersect.hpp"

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

The project uses Google Test framework for comprehensive unit testing.

```bash
# Build with tests enabled
cmake .. -DGEOM_BUILD_TESTS=ON
cmake --build .

# Run all tests  
ctest --output-on-failure

# Run tests with verbose output
ctest --verbose

# Run specific test suites
./tests/geometry_gtests --gtest_filter="RayTest.*"
./tests/geometry_gtests --gtest_filter="TriangleTest.*"  
./tests/geometry_gtests --gtest_filter="IntersectionTest.*"
```

## Project Structure

```
geometry/
├── CMakeLists.txt        # Main project configuration
├── include/geom/         # Public API headers  
│   ├── core/            # Core configuration
│   │   └── config.hpp   # Scalar types and config
│   ├── primitives/      # Geometric primitives
│   │   ├── ray.hpp     # Ray3d definition
│   │   └── triangle.hpp # Triangle3d definition
│   └── algorithms/      # Geometric algorithms
│       └── intersect.hpp # Intersection algorithms
├── src/                 # Implementation files
│   ├── core/           
│   │   └── CMakeLists.txt # Core module config
│   ├── primitives/      
│   │   ├── CMakeLists.txt # Primitives module config
│   │   ├── ray.cpp     # Ray implementations
│   │   └── triangle.cpp # Triangle implementations
│   └── algorithms/     
│       ├── CMakeLists.txt # Algorithms module config
│       └── intersect.cpp  # Intersection implementations
├── tests/               # Google Test suite
│   ├── CMakeLists.txt  # Test configuration
│   ├── test_primitives_gtest.cpp # Primitives tests
│   └── test_intersect_gtest.cpp  # Intersection tests
├── .github/workflows/   # CI/CD automation
├── cmake/              # CMake configuration
└── docs/               # Documentation
```

## CMake Architecture

The project uses a modular CMake structure:

- **Root CMakeLists.txt**: Main project coordination and global settings
- **src/*/CMakeLists.txt**: Dedicated module configurations
- **tests/CMakeLists.txt**: Google Test integration with CTest

### Available Targets

- `geom_core`: Interface library with Eigen and configuration
- `geom_primitives`: Static library with Ray3d and Triangle3d  
- `geom_algorithms`: Static library with intersection algorithms
- `geom`: Main library target combining all modules
- `geometry_gtests`: Google Test executable

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) for development guidelines.

## License

See [LICENSE](LICENSE) file for details.
