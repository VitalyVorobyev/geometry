# Spatial data structures
add_library(geom_spatial INTERFACE)

target_include_directories(geom_spatial INTERFACE
    $<BUILD_INTERFACE:${GEOM_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

# Link dependencies
# None for now

add_library(geometry::spatial ALIAS geom_spatial)
