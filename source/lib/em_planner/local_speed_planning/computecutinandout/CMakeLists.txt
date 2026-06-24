add_library(computecutinandout STATIC
    computecutinandout.cpp)

target_include_directories(computecutinandout PUBLIC
    ${CMAKE_CURRENT_LIST_DIR})

target_compile_features(computecutinandout PUBLIC cxx_std_17)

target_link_libraries(computecutinandout PUBLIC
    em_planner_perception
    localpathreferline)

add_executable(test_compute_cut_in_and_out_smoke EXCLUDE_FROM_ALL
    test_compute_cut_in_and_out_smoke.cpp)
target_link_libraries(test_compute_cut_in_and_out_smoke PRIVATE
    computecutinandout)
