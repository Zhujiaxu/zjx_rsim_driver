add_library(map_helper STATIC
    MapHelper.cpp)

target_include_directories(map_helper PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(map_helper PUBLIC
    driver_common
    OpenDriveParser)

# Optional standalone smoke binary for quick API checks.
add_executable(test_map_helper_smoke EXCLUDE_FROM_ALL test_map_helper_smoke.cpp)
target_link_libraries(test_map_helper_smoke PRIVATE map_helper)
