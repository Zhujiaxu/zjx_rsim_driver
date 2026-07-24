if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    cmake_minimum_required(VERSION 3.16)
    project(global_path LANGUAGES CXX)
endif()

add_library(global_path STATIC
    GlobalPathGenerator.cpp)

target_include_directories(global_path PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR})

target_compile_features(global_path PUBLIC cxx_std_17)

target_link_libraries(global_path PUBLIC
    map_helper
    logwriter
    pugixml::pugixml)

add_executable(test_global_path_smoke EXCLUDE_FROM_ALL
    test_global_path_smoke.cpp)
target_link_libraries(test_global_path_smoke PRIVATE
    global_path)
