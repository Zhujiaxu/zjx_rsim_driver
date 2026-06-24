if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    cmake_minimum_required(VERSION 3.16)
    project(common LANGUAGES CXX)
endif()

add_library(pathmatcher INTERFACE)
target_include_directories(pathmatcher INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_features(pathmatcher INTERFACE cxx_std_17)

add_executable(test_path_matcher_smoke EXCLUDE_FROM_ALL
    test_path_matcher_smoke.cpp)
target_link_libraries(test_path_matcher_smoke PRIVATE
    pathmatcher)
