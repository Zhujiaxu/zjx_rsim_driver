add_library(em_planner_perception SHARED
    FrenetObstaclePerception.cpp)

target_include_directories(em_planner_perception PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    ${RSIM_PLUGIN_SDK_INCLUDE})

target_compile_features(em_planner_perception PUBLIC cxx_std_17)

target_link_libraries(em_planner_perception PUBLIC
    cartesian_to_frenet)

if(BUILD_TESTING)
    add_executable(test_frenet_obstacle_perception_smoke
        test_frenet_obstacle_perception_smoke.cpp)
    target_link_libraries(test_frenet_obstacle_perception_smoke PRIVATE
        em_planner_perception)
    add_test(NAME frenet_obstacle_perception_smoke
        COMMAND test_frenet_obstacle_perception_smoke)
endif()
