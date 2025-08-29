# Algorithms library
add_library(geom_algorithms STATIC)

# Source files
target_sources(geom_algorithms PRIVATE
    intersect.cpp
)

# Include directories
target_include_directories(geom_algorithms PUBLIC
    $<BUILD_INTERFACE:${GEOM_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

# Link dependencies
target_link_libraries(geom_algorithms PUBLIC geom_primitives)

# Alias for consistent naming
add_library(geometry::algorithms ALIAS geom_algorithms)
