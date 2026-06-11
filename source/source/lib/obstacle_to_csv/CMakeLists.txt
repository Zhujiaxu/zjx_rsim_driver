if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    cmake_minimum_required(VERSION 3.16)
    project(obstacle_to_csv LANGUAGES CXX)
endif()

add_library(obstacle_to_csv STATIC
    ObstacleToCsv.cpp)

target_include_directories(obstacle_to_csv PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${RSIM_PLUGIN_SDK_INCLUDE})

target_compile_features(obstacle_to_csv PUBLIC cxx_std_17)

add_executable(test_obstacle_to_csv_smoke EXCLUDE_FROM_ALL
    test_obstacle_to_csv_smoke.cpp)
target_link_libraries(test_obstacle_to_csv_smoke PRIVATE
    obstacle_to_csv)
