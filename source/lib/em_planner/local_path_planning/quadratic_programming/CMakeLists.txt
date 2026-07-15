add_library(quadratic_programming STATIC
    QpPathOptimizer.cpp)

target_include_directories(quadratic_programming PUBLIC
    ${CMAKE_CURRENT_LIST_DIR})
    
target_compile_features(quadratic_programming PUBLIC cxx_std_17)

target_link_libraries(quadratic_programming PUBLIC
    OsqpEigen::OsqpEigen
    dynamic_programming
    drivable_area
    em_planner_perception
    frenet_to_cartesian)

add_executable(test_qp_path_optimizer_smoke EXCLUDE_FROM_ALL
    test_qp_path_optimizer_smoke.cpp)
target_link_libraries(test_qp_path_optimizer_smoke PRIVATE
    quadratic_programming)
