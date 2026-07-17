add_library(st_drivable_area STATIC
    StDrivableArea.cpp)

target_include_directories(st_drivable_area PUBLIC
    ${CMAKE_CURRENT_LIST_DIR})

target_compile_features(st_drivable_area PUBLIC cxx_std_17)

target_link_libraries(st_drivable_area PUBLIC
    collision_cost
    computecutinandout
    dynamic_speed_planning)

add_executable(test_st_drivable_area_smoke EXCLUDE_FROM_ALL
    test_drivable_area_smoke.cpp)
target_link_libraries(test_st_drivable_area_smoke PRIVATE
    st_drivable_area)
