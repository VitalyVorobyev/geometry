# Core library
add_library(geom_core INTERFACE)
target_compile_features(geom_core INTERFACE cxx_std_20)
target_include_directories(geom_core INTERFACE 
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(geom_core INTERFACE Eigen3::Eigen)

# Compiler warnings
if(MSVC)
    target_compile_options(geom_core INTERFACE /permissive- /Zc:__cplusplus /W4 /WX-)
else()
    target_compile_options(geom_core INTERFACE -Wall -Wextra -Wpedantic -Wshadow -Wconversion)
endif()

# Preprocessor definitions
if(GEOM_USE_DOUBLE)
    target_compile_definitions(geom_core INTERFACE GEOM_SCALAR_DOUBLE=1)
endif()

# Primitives module
add_library(geom_primitives STATIC)
target_sources(geom_primitives PRIVATE
    primitives/ray.cpp
    primitives/triangle.cpp
)
target_link_libraries(geom_primitives PUBLIC geom_core)

# Algorithms module
add_library(geom_algorithms STATIC)
target_sources(geom_algorithms PRIVATE
    algorithms/intersect.cpp
)
target_link_libraries(geom_algorithms PUBLIC geom_primitives)

# Main geometry library (combines all modules)
add_library(geom INTERFACE)
target_link_libraries(geom INTERFACE geom_core geom_primitives geom_algorithms)

# Alias for consistent naming
add_library(geometry::geom ALIAS geom)