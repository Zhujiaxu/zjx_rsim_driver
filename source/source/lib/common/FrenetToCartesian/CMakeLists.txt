if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    cmake_minimum_required(VERSION 3.16)
    project(frenet_to_cartesian LANGUAGES CXX)
endif()

add_library(frenet_to_cartesian INTERFACE)
target_include_directories(frenet_to_cartesian INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_features(frenet_to_cartesian INTERFACE cxx_std_17)

add_executable(test_frenet_to_cartesian_smoke EXCLUDE_FROM_ALL
    test_frenet_to_cartesian_smoke.cpp)
target_link_libraries(test_frenet_to_cartesian_smoke PRIVATE
    frenet_to_cartesian)
