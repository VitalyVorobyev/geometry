# Primitives library
add_library(geom_primitives STATIC)

# Source files
target_sources(geom_primitives PRIVATE
    ray.cpp
    triangle.cpp
)

# Include directories
target_include_directories(geom_primitives PUBLIC
    $<BUILD_INTERFACE:${GEOM_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

# Link dependencies
target_link_libraries(geom_primitives PUBLIC geom_core)

# Alias for consistent naming
add_library(geometry::primitives ALIAS geom_primitives)
