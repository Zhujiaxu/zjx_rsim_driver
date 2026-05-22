add_library(perception_helper STATIC
    PerceptionHelper.cpp)

target_include_directories(perception_helper PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${RSIM_PLUGIN_SDK_INCLUDE})

target_link_libraries(perception_helper PUBLIC
    driver_common)
