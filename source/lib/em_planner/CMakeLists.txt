if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    cmake_minimum_required(VERSION 3.16)
    project(em_planner LANGUAGES CXX)
endif()

add_library(em_planner STATIC
    planning_start/PlanningStartPoint.cpp)

target_include_directories(em_planner PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR})

target_compile_features(em_planner PUBLIC cxx_std_17)

add_executable(test_planning_start_point EXCLUDE_FROM_ALL
    planning_start/test_planning_start_point.cpp)
target_link_libraries(test_planning_start_point PRIVATE
    em_planner)
