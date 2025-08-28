#!/bin/bash

# Build script for geometry library
set -e

# Parse command line arguments
BUILD_TYPE="Release"
USE_DOUBLE="ON"
BUILD_TESTS="ON"
BUILD_EXAMPLES="OFF"
CLEAN_BUILD="false"

while [[ $# -gt 0 ]]; do
    case $1 in
        --debug)
            BUILD_TYPE="Debug"
            shift
            ;;
        --float)
            USE_DOUBLE="OFF"
            shift
            ;;
        --no-tests)
            BUILD_TESTS="OFF"
            shift
            ;;
        --examples)
            BUILD_EXAMPLES="ON"
            shift
            ;;
        --clean)
            CLEAN_BUILD="true"
            shift
            ;;
        --help)
            echo "Usage: $0 [options]"
            echo "Options:"
            echo "  --debug       Build in Debug mode (default: Release)"
            echo "  --float       Use float precision (default: double)"
            echo "  --no-tests    Don't build tests (default: build tests)"
            echo "  --examples    Build examples (default: off)"
            echo "  --clean       Clean build directory first"
            echo "  --help        Show this help"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$SCRIPT_DIR/build"

# Clean build if requested
if [ "$CLEAN_BUILD" = "true" ]; then
    echo "Cleaning build directory..."
    rm -rf "$BUILD_DIR"
fi

# Create build directory
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

# Configure
echo "Configuring with:"
echo "  Build type: $BUILD_TYPE"
echo "  Use double: $USE_DOUBLE"
echo "  Build tests: $BUILD_TESTS"
echo "  Build examples: $BUILD_EXAMPLES"

cmake .. \
    -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
    -DGEOM_USE_DOUBLE="$USE_DOUBLE" \
    -DGEOM_BUILD_TESTS="$BUILD_TESTS" \
    -DGEOM_BUILD_EXAMPLES="$BUILD_EXAMPLES"

# Build
echo "Building..."
cmake --build . --parallel

# Run tests if enabled
if [ "$BUILD_TESTS" = "ON" ]; then
    echo "Running tests..."
    ctest --output-on-failure
fi

echo "Build completed successfully!"
