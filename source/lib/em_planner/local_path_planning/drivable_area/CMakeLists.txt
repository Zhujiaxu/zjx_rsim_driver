add_library(drivable_area STATIC
    DrivableArea.cpp)

target_include_directories(drivable_area PUBLIC
    ${CMAKE_CURRENT_LIST_DIR})

target_compile_features(drivable_area PUBLIC cxx_std_17)

target_link_libraries(drivable_area PUBLIC
    collision_cost
    dynamic_programming)

add_executable(test_drivable_area_smoke EXCLUDE_FROM_ALL
    test_drivable_area_smoke.cpp)
target_link_libraries(test_drivable_area_smoke PRIVATE
    drivable_area)
