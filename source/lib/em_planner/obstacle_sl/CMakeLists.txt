add_library(obstacle_sl STATIC
    ObstacleSlConverter.cpp)

target_include_directories(obstacle_sl PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/..)

target_compile_features(obstacle_sl PUBLIC cxx_std_17)

target_link_libraries(obstacle_sl PUBLIC
    cartesian_to_frenet
    em_planner_perception)

add_executable(test_obstacle_sl_converter_smoke EXCLUDE_FROM_ALL
    test_obstacle_sl_converter_smoke.cpp)
target_link_libraries(test_obstacle_sl_converter_smoke PRIVATE
    obstacle_sl)
