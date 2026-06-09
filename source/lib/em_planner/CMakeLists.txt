if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    cmake_minimum_required(VERSION 3.16)
    project(em_planner LANGUAGES CXX)
endif()

if(NOT DEFINED RSIM_PLUGIN_SDK_INCLUDE)
    set(RSIM_PLUGIN_SDK_INCLUDE
        ${CMAKE_CURRENT_LIST_DIR}/../../../third_party/RSimAPI/source)
endif()

add_subdirectory(planning_start)
add_subdirectory(perception)
add_subdirectory(obstacle_sl)
add_subdirectory(dynamic_programming)

add_library(em_planner STATIC
    EmPlanner.cpp)

target_link_libraries(em_planner PUBLIC
    planning_start
    em_planner_perception
    obstacle_sl
    dynamic_programming)
