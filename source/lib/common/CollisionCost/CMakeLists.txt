if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    cmake_minimum_required(VERSION 3.16)
    project(collision_cost LANGUAGES CXX)
endif()

add_library(collision_cost INTERFACE)
target_include_directories(collision_cost INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_features(collision_cost INTERFACE cxx_std_17)

add_executable(test_collision_cost_smoke EXCLUDE_FROM_ALL
    test_collision_cost_smoke.cpp)
target_link_libraries(test_collision_cost_smoke PRIVATE
    collision_cost)
