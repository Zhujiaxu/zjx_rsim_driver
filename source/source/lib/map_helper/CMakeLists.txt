add_library(map_helper STATIC
    MapHelper.cpp)

target_include_directories(map_helper PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(map_helper PUBLIC
    OpenDriveParser)

# Optional standalone smoke binary for quick API checks.
# (disabled — test_map_helper_smoke.cpp removed)
