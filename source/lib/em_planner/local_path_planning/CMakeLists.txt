add_subdirectory(dynamic_programming)
add_subdirectory(drivable_area)
add_subdirectory(quadratic_programming)
add_subdirectory(dp_increase_points)


add_library(local_path_planning INTERFACE)

target_compile_features(local_path_planning INTERFACE cxx_std_17)

target_link_libraries(local_path_planning INTERFACE
    dynamic_programming
    drivable_area
    quadratic_programming
    dpincreasepoints)
