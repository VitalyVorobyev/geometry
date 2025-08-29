# Google Test based unit tests
add_executable(geometry_gtests)

# Test sources
target_sources(geometry_gtests PRIVATE
    test_primitives_gtest.cpp
    test_intersect_gtest.cpp
)

# Link with geometry library and GTest
target_link_libraries(geometry_gtests PRIVATE
    geom
    GTest::GTest
    GTest::Main
)

# Include project root for proper includes
target_include_directories(geometry_gtests PRIVATE ${GEOM_INCLUDE_DIR})

# Register with CTest
include(GoogleTest)
gtest_discover_tests(geometry_gtests)
