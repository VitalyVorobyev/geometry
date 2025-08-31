add_executable(basic_example basic_example.cpp)
target_link_libraries(basic_example PRIVATE geom)

add_executable(spatial_bench spatial_bench.cpp)
target_link_libraries(spatial_bench PRIVATE geom)
