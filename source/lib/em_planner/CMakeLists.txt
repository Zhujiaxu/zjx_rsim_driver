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
add_subdirectory(local_path_planning)
add_subdirectory(local_speed_planning)

add_library(em_planner SHARED
    EmPlanner.cpp)

target_include_directories(em_planner PUBLIC
    ${CMAKE_CURRENT_LIST_DIR})

target_compile_features(em_planner PUBLIC cxx_std_17)

target_link_libraries(em_planner PUBLIC
    planning_start
    em_planner_perception
    local_path_planning
    local_speed_planning)
