cmake_minimum_required(VERSION 3.22)
project(geometry LANGUAGES CXX VERSION 0.1.0)

# Options
option(GEOM_USE_DOUBLE "Use double as default scalar" ON)
option(GEOM_BUILD_TESTS "Build unit tests" ON)
option(GEOM_BUILD_EXAMPLES "Build example programs" OFF)

# Global compile features and standards
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(Eigen3 3.3 REQUIRED)

# Find GTest for testing
if(GEOM_BUILD_TESTS)
    find_package(GTest REQUIRED)
    enable_testing()
endif()

# Set project-wide include directory
# Include directory - point to include folder for geom/* style includes
set(GEOM_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Global compiler definitions
if(GEOM_USE_DOUBLE)
    add_compile_definitions(GEOM_SCALAR_DOUBLE=1)
endif()

# Global compiler warnings
if(MSVC)
    add_compile_options(/permissive- /Zc:__cplusplus /W4 /WX-)
else()
    add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion)
endif()

# Add subdirectories with modules
add_subdirectory(src/core)
add_subdirectory(src/primitives)
add_subdirectory(src/algorithms)

# Tests
if(GEOM_BUILD_TESTS)
    add_subdirectory(tests)
endif()

# Examples
if(GEOM_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# Main geometry library (combines all modules) - defined after all modules
add_library(geom INTERFACE)
target_link_libraries(geom INTERFACE
    geom_core
    geom_primitives
    geom_algorithms
)
add_library(geometry::geom ALIAS geom)

# Packaging
include(GNUInstallDirs)

# Install targets
install(TARGETS geom geom_core geom_primitives geom_algorithms
    EXPORT geometry-targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Install headers
install(DIRECTORY include/geom
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    FILES_MATCHING PATTERN "*.hpp"
)

# Install CMake config files
install(EXPORT geometry-targets
    FILE geometry-targets.cmake
    NAMESPACE geometry::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/geometry
)

# Create config file
include(CMakePackageConfigHelpers)
configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/geometry-config.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/geometry-config.cmake"
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/geometry
)

# CPack configuration
set(CPACK_PACKAGE_NAME "geometry")
set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "A C++ geometry library with ray-triangle intersection")
set(CPACK_PACKAGE_VENDOR "VitalyVorobyev")
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
set(CPACK_RESOURCE_FILE_README "${CMAKE_CURRENT_SOURCE_DIR}/README.md")
include(CPack)
