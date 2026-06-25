add_library(dynamic_speed_planning STATIC
    DynamicSpeedPlanner.cpp)

target_include_directories(dynamic_speed_planning PUBLIC
    ${CMAKE_CURRENT_LIST_DIR})

target_compile_features(dynamic_speed_planning PUBLIC cxx_std_17)

target_link_libraries(dynamic_speed_planning PUBLIC
    planning_start
    collision_cost
    em_planner_perception
    point_to_line_distance
    localpathreferline
    )

add_executable(test_dynamic_speed_planner_smoke EXCLUDE_FROM_ALL
    test_dynamic_speed_planner_smoke.cpp)
target_link_libraries(test_dynamic_speed_planner_smoke PRIVATE
    dynamic_speed_planning)
