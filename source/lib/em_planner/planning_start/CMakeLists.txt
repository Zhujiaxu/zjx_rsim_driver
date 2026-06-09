add_library(planning_start STATIC
    PlanningStartPoint.cpp)

target_include_directories(planning_start PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/..)

target_compile_features(planning_start PUBLIC cxx_std_17)

target_link_libraries(planning_start PUBLIC
    cartesian_to_frenet)

add_executable(test_planning_start_point EXCLUDE_FROM_ALL
    test_planning_start_point.cpp)
target_link_libraries(test_planning_start_point PRIVATE
    planning_start)

add_executable(test_planning_start_frenet_smoke EXCLUDE_FROM_ALL
    test_planning_start_frenet_smoke.cpp)
target_link_libraries(test_planning_start_frenet_smoke PRIVATE
    planning_start)
