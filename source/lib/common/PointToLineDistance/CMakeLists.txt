if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    cmake_minimum_required(VERSION 3.16)
    project(point_to_line_distance LANGUAGES CXX)
endif()

add_library(point_to_line_distance INTERFACE)
target_include_directories(point_to_line_distance INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_features(point_to_line_distance INTERFACE cxx_std_17)

add_executable(test_point_to_line_distance_smoke EXCLUDE_FROM_ALL
    test_point_to_line_distance_smoke.cpp)
target_link_libraries(test_point_to_line_distance_smoke PRIVATE
    point_to_line_distance)
