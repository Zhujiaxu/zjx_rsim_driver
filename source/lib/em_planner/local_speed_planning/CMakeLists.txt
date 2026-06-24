
add_subdirectory(localpathreferline)
add_subdirectory(dynamicobstaclefrenetinfo)

add_library(local_speed_planning INTERFACE)

target_compile_features(local_speed_planning INTERFACE cxx_std_17)

target_link_libraries(local_speed_planning INTERFACE
    localpathreferline
    dynamicobsfrenet)
