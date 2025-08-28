# Individual test executables for modular testing
add_executable(test_intersect_only)
target_sources(test_intersect_only PRIVATE
    test_intersect.cpp
    test_intersect_main.cpp
)
target_link_libraries(test_intersect_only PRIVATE geom)
add_test(NAME intersect_tests COMMAND test_intersect_only)

add_executable(test_primitives_only)  
target_sources(test_primitives_only PRIVATE
    test_primitives.cpp
    test_primitives_main.cpp
)
target_link_libraries(test_primitives_only PRIVATE geom)
add_test(NAME primitives_tests COMMAND test_primitives_only)

# Main comprehensive test executable
add_executable(geometry_tests)
target_sources(geometry_tests PRIVATE
    main_test_runner.cpp
    test_primitives.cpp
    test_intersect.cpp
)
target_link_libraries(geometry_tests PRIVATE geom)
add_test(NAME geometry_all_tests COMMAND geometry_tests)
