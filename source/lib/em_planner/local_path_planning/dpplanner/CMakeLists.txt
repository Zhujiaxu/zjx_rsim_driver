add_library(dpplanner STATIC
    DpPlanner.cpp)

target_include_directories(dpplanner PUBLIC
    ${CMAKE_CURRENT_LIST_DIR})
    
target_compile_features(dpplanner PUBLIC cxx_std_17)

target_link_libraries(dpplanner PUBLIC
    cartesian_to_frenet
    collision_cost
    em_planner_perception
    quintic_polynomial)

add_executable(test_dynamic_programming_planner_smoke EXCLUDE_FROM_ALL
    test_dynamic_programming_planner_smoke.cpp)
target_link_libraries(test_dynamic_programming_planner_smoke PRIVATE
    dpplanner)
