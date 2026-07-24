if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    cmake_minimum_required(VERSION 3.16)
    project(cartesian_to_frenet LANGUAGES CXX)
endif()

add_library(cartesian_to_frenet INTERFACE)
target_include_directories(cartesian_to_frenet INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_features(cartesian_to_frenet INTERFACE cxx_std_17)
target_link_libraries(cartesian_to_frenet INTERFACE
    pathmatcher
    logwriter)

add_executable(test_cartesian_to_frenet_smoke EXCLUDE_FROM_ALL
    test_cartesian_to_frenet_smoke.cpp)
target_link_libraries(test_cartesian_to_frenet_smoke PRIVATE
    cartesian_to_frenet)
