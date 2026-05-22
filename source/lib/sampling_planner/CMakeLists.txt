add_library(sampling_planner STATIC
    SamplingPlanner.cpp)

target_include_directories(sampling_planner PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(sampling_planner PUBLIC
    driver_common)

add_executable(test_sampling_planner_smoke EXCLUDE_FROM_ALL
    test_sampling_planner_smoke.cpp)
target_link_libraries(test_sampling_planner_smoke PRIVATE
    sampling_planner)
