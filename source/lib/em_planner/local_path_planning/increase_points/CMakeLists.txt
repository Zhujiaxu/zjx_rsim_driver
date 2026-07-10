add_library(increasepoints STATIC increasepoints.cpp)
target_link_libraries(increasepoints PUBLIC
    dynamic_programming
    quintic_polynomial)
target_include_directories(increasepoints PUBLIC
    ${CMAKE_CURRENT_LIST_DIR})

add_executable(test_increase_points_smoke EXCLUDE_FROM_ALL
    test_increase_points_smoke.cpp)
target_link_libraries(test_increase_points_smoke PRIVATE
    increasepoints)