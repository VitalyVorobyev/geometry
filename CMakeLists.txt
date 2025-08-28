cmake_minimum_required(VERSION 3.22)
project(geometry LANGUAGES CXX VERSION 0.1.0)

# Options
option(GEOM_USE_DOUBLE "Use double as default scalar" ON)
option(GEOM_BUILD_TESTS "Build unit tests" ON)
option(GEOM_BUILD_EXAMPLES "Build example programs" OFF)

# Global compile features
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(Eigen3 3.3 REQUIRED)

# Add subdirectories with modules
add_subdirectory(src)

# Tests
if(GEOM_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# Examples
if(GEOM_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

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
