add_library(dpspeedplanner STATIC
    DynamicSpeedPlanner.cpp)

target_include_directories(dpspeedplanner PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/../../perception)

target_compile_features(dpspeedplanner PUBLIC cxx_std_17)

target_link_libraries(dpspeedplanner PUBLIC
    planning_start
    collision_cost
    em_planner_perception
    localpathreferline
    computecutinandout
    )

add_executable(test_dynamic_speed_planner_smoke EXCLUDE_FROM_ALL
    test_dynamic_speed_planner_smoke.cpp)
target_link_libraries(test_dynamic_speed_planner_smoke PRIVATE
    dpspeedplanner)
