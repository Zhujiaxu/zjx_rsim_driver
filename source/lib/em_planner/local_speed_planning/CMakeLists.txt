
add_subdirectory(localpathreferline)
add_subdirectory(computecutinandout)
add_subdirectory(dynamic_speed_planning)

add_library(local_speed_planning INTERFACE)

target_compile_features(local_speed_planning INTERFACE cxx_std_17)

target_link_libraries(local_speed_planning INTERFACE
    localpathreferline
    computecutinandout
    dynamic_speed_planning)
