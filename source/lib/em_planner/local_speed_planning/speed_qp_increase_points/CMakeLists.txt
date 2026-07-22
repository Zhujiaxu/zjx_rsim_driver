add_library(speed_qp_increase_points STATIC
    SpeedQpIncreasePoints.cpp)

target_include_directories(speed_qp_increase_points PUBLIC
    ${CMAKE_CURRENT_LIST_DIR})

target_compile_features(speed_qp_increase_points PUBLIC cxx_std_17)

target_link_libraries(speed_qp_increase_points PUBLIC
    speed_quadratic_programming)

if(BUILD_TESTING)
    add_executable(test_speed_qp_increase_points_smoke EXCLUDE_FROM_ALL
        test_speed_qp_increase_points_smoke.cpp)
    target_link_libraries(test_speed_qp_increase_points_smoke PRIVATE
        speed_qp_increase_points)
    add_test(NAME speed_qp_increase_points_smoke
        COMMAND test_speed_qp_increase_points_smoke)
endif()
