if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    cmake_minimum_required(VERSION 3.16)
    project(logwriter LANGUAGES CXX)
endif()

add_library(logwriter INTERFACE)
target_include_directories(logwriter INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_features(logwriter INTERFACE cxx_std_17)
