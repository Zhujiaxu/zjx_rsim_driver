add_library(localpathreferline INTERFACE)

target_include_directories(localpathreferline INTERFACE
    ${CMAKE_CURRENT_LIST_DIR})

target_compile_features(localpathreferline INTERFACE cxx_std_17)

target_link_libraries(localpathreferline INTERFACE
    local_path_planning)

add_executable(test_local_reference_line_path_smoke EXCLUDE_FROM_ALL
    test_local_reference_line_path_smoke.cpp)
target_link_libraries(test_local_reference_line_path_smoke PRIVATE
    localpathreferline)
