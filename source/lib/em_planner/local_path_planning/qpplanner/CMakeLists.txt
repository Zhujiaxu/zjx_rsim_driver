add_library(qpplanner STATIC
    QpPathOptimizer.cpp)

target_include_directories(qpplanner PUBLIC
    ${CMAKE_CURRENT_LIST_DIR})
    
target_compile_features(qpplanner PUBLIC cxx_std_17)

target_link_libraries(qpplanner PUBLIC
    OsqpEigen::OsqpEigen
    dpplanner
    drivable_area
    em_planner_perception
    frenet_to_cartesian)

add_executable(test_qp_path_optimizer_smoke EXCLUDE_FROM_ALL
    test_qp_path_optimizer_smoke.cpp)
target_link_libraries(test_qp_path_optimizer_smoke PRIVATE
    qpplanner
    qpincreasepoints)
