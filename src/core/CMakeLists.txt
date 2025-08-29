# Core library
add_library(geom_core INTERFACE)

# Set target properties
target_compile_features(geom_core INTERFACE cxx_std_20)

# Include directories
target_include_directories(geom_core INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

# Link with Eigen3
target_link_libraries(geom_core INTERFACE Eigen3::Eigen)

# Alias for consistent naming
add_library(geometry::core ALIAS geom_core)
