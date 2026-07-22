
add_subdirectory(localpathreferline)
add_subdirectory(computecutinandout)
add_subdirectory(dynamic_speed_planning)
add_subdirectory(speed_drivable_area)
add_subdirectory(speed_quadratic_programming)
add_subdirectory(speed_qp_increase_points)

add_library(local_speed_planning INTERFACE)

target_compile_features(local_speed_planning INTERFACE cxx_std_17)

target_link_libraries(local_speed_planning INTERFACE
    localpathreferline
    computecutinandout
    dynamic_speed_planning
    st_drivable_area
    speed_quadratic_programming
    speed_qp_increase_points)
