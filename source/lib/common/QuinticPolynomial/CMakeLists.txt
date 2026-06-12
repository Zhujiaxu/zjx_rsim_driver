if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    cmake_minimum_required(VERSION 3.16)
    project(quintic_polynomial LANGUAGES CXX)
endif()

if(NOT TARGET Eigen3::Eigen)
    set(RSIM_DRIVER_EIGEN_INCLUDE
        ${CMAKE_CURRENT_LIST_DIR}/../../../../third_party/eigen/include
        CACHE PATH "Eigen3 include directory")
    add_library(Eigen3_Eigen INTERFACE)
    target_include_directories(Eigen3_Eigen INTERFACE
        ${RSIM_DRIVER_EIGEN_INCLUDE})
    add_library(Eigen3::Eigen ALIAS Eigen3_Eigen)
endif()

add_library(quintic_polynomial INTERFACE)
target_include_directories(quintic_polynomial INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_features(quintic_polynomial INTERFACE cxx_std_17)
target_link_libraries(quintic_polynomial INTERFACE
    Eigen3::Eigen)

add_executable(test_quintic_polynomial_smoke EXCLUDE_FROM_ALL
    test_quintic_polynomial_smoke.cpp)
target_link_libraries(test_quintic_polynomial_smoke PRIVATE
    quintic_polynomial)
