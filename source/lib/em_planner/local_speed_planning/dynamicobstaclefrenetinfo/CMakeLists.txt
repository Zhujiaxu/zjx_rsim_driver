add_library(dynamicobsfrenet STATIC dynamicobsfrenet.cpp)
target_link_libraries(dynamicobsfrenet PUBLIC
    em_planner_perception
    localpathreferline)

add_executable(test_dynamic_obstacle_frenet_info_smoke EXCLUDE_FROM_ALL
    test_dynamic_obstacle_frenet_info_smoke.cpp)
target_link_libraries(test_dynamic_obstacle_frenet_info_smoke PRIVATE
    dynamicobsfrenet)
