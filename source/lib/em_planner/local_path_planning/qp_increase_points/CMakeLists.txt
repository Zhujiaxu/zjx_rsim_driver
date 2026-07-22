add_library(qpincreasepoints STATIC qpincreasepoints.cpp)

target_include_directories(qpincreasepoints PUBLIC
    ${CMAKE_CURRENT_LIST_DIR})

target_compile_features(qpincreasepoints PUBLIC cxx_std_17)

target_link_libraries(qpincreasepoints PUBLIC
    qpplanner)

add_executable(test_qp_increase_points_smoke EXCLUDE_FROM_ALL
    test_qp_increase_points_smoke.cpp)

target_link_libraries(test_qp_increase_points_smoke PRIVATE
    qpincreasepoints)
