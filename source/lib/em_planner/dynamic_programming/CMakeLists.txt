add_library(dynamic_programming STATIC
    DpPlanner.cpp)

target_include_directories(dynamic_programming PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/..)

target_compile_features(dynamic_programming PUBLIC cxx_std_17)

target_link_libraries(dynamic_programming PUBLIC
    collision_cost
    quintic_polynomial
    obstacle_sl
    planning_start_sl)

add_executable(test_dynamic_programming_planner_smoke EXCLUDE_FROM_ALL
    test_dynamic_programming_planner_smoke.cpp)
target_link_libraries(test_dynamic_programming_planner_smoke PRIVATE
    dynamic_programming)
