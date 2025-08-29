# Contributing to Geometry Library

## Development Setup

### Prerequisites
- CMake 3.22 or higher
- C++20 compatible compiler (GCC 10+, Clang 10+, MSVC 2019+)
- Eigen3 library (version 3.3+)

### Building from Source

```bash
# Clone the repository
git clone https://github.com/VitalyVorobyev/geometry.git
cd geometry

# Configure and build
mkdir build && cd build
cmake .. -DGEOM_BUILD_TESTS=ON -DGEOM_USE_DOUBLE=ON
cmake --build .

# Run tests
ctest --output-on-failure
```

### CMake Options

- `GEOM_USE_DOUBLE`: Use double precision floating point (default: ON)
- `GEOM_BUILD_TESTS`: Build unit tests (default: ON)

## Project Structure

The project is organized into modular CMake targets:

- **geom_core**: Core library with configuration and types
- **geom_primitives**: Geometric primitives (Ray3d, Triangle3d)  
- **geom_algorithms**: Geometric algorithms (intersection, etc.)
- **geom**: Main library target that combines all modules

### Directory Structure

```
geometry/
├── include/geom/          # Public headers
│   ├── core/              # Core types and configuration
│   ├── primitives/        # Geometric primitives
│   └── algorithms/        # Geometric algorithms
├── src/                   # Implementation files
│   ├── core/              
│   ├── primitives/        
│   └── algorithms/        
├── tests/                 # Unit tests
├── .github/workflows/     # CI/CD configuration
└── cmake/                 # CMake configuration files
```

## Code Style

- Follow C++20 best practices
- Use consistent indentation (4 spaces)
- Document public APIs with doxygen-style comments
- Keep headers self-contained with proper includes
- Use namespace `geom` for all library code

## Testing

- Write unit tests for all new functionality
- Tests are organized by module (primitives, algorithms, etc.)
- Run individual test suites: `ctest -R primitives_tests`
- Run all tests: `ctest`

## Submitting Changes

1. Fork the repository
2. Create a feature branch: `git checkout -b feature-name`
3. Make your changes and add tests
4. Ensure all tests pass: `cmake --build . && ctest`
5. Submit a pull request

All pull requests are automatically tested on multiple platforms via GitHub Actions.
