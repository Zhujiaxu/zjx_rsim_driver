if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    cmake_minimum_required(VERSION 3.16)
    project(quintic_polynomial LANGUAGES CXX)
endif()

add_library(quintic_polynomial INTERFACE)
target_include_directories(quintic_polynomial INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_features(quintic_polynomial INTERFACE cxx_std_17)

add_executable(test_quintic_polynomial_smoke EXCLUDE_FROM_ALL
    test_quintic_polynomial_smoke.cpp)
target_link_libraries(test_quintic_polynomial_smoke PRIVATE
    quintic_polynomial)
