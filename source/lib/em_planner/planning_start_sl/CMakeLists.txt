add_library(planning_start_sl INTERFACE)

target_include_directories(planning_start_sl INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/..)

target_compile_features(planning_start_sl INTERFACE cxx_std_17)

target_link_libraries(planning_start_sl INTERFACE
    cartesian_to_frenet
    planning_start)

add_executable(test_planning_start_sl_smoke EXCLUDE_FROM_ALL
    test_planning_start_sl_smoke.cpp)
target_link_libraries(test_planning_start_sl_smoke PRIVATE
    planning_start_sl)
