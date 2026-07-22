add_subdirectory(dpplanner)
add_subdirectory(drivable_area)
add_subdirectory(qp_increase_points)
add_subdirectory(qpplanner)
add_subdirectory(dp_increase_points)


add_library(local_path_planning INTERFACE)

target_compile_features(local_path_planning INTERFACE cxx_std_17)

target_link_libraries(local_path_planning INTERFACE
    dpplanner
    drivable_area
    qpplanner
    qpincreasepoints
    dpincreasepoints)
