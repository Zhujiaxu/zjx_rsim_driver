add_library(em_planner STATIC
    EMPlanner.cpp
    DpPathOptimizer.cpp
    QpPathOptimizer.cpp
    DpSpeedOptimizer.cpp
    QpSpeedOptimizer.cpp
    StGraph.cpp
    PathTimeTrajectory.cpp)

target_include_directories(em_planner PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(em_planner PUBLIC
    driver_common
    reference_line
    OsqpEigen::OsqpEigen)

add_executable(test_em_planner_smoke EXCLUDE_FROM_ALL
    test_em_planner_smoke.cpp)
target_link_libraries(test_em_planner_smoke PRIVATE
    em_planner)
