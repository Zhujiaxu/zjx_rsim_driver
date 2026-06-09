add_library(em_planner_perception STATIC
    StaticObstacleDetector.cpp)

target_include_directories(em_planner_perception PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/..
    ${RSIM_PLUGIN_SDK_INCLUDE})

target_compile_features(em_planner_perception PUBLIC cxx_std_17)

add_executable(test_static_obstacle_detector_smoke EXCLUDE_FROM_ALL
    test_static_obstacle_detector_smoke.cpp)
target_link_libraries(test_static_obstacle_detector_smoke PRIVATE
    em_planner_perception)
