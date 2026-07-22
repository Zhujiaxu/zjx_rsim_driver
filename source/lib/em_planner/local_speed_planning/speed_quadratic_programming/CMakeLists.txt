add_library(speed_quadratic_programming STATIC
    SpeedQpOptimizer.cpp)

target_include_directories(speed_quadratic_programming PUBLIC
    ${CMAKE_CURRENT_LIST_DIR})

target_compile_features(speed_quadratic_programming PUBLIC cxx_std_17)

target_link_libraries(speed_quadratic_programming PUBLIC
    OsqpEigen::OsqpEigen
    collision_cost
    dpspeedplanner
    st_drivable_area)

add_executable(test_speed_qp_optimizer_smoke EXCLUDE_FROM_ALL
    test_speed_qp_optimizer_smoke.cpp)
target_link_libraries(test_speed_qp_optimizer_smoke PRIVATE
    speed_quadratic_programming)
