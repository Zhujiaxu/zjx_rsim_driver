add_library(reference_line STATIC
    ReferenceLineGenerator.cpp
    ReferenceLineSmoother.cpp)

target_include_directories(reference_line PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(reference_line PUBLIC
    OsqpEigen::OsqpEigen)

add_executable(test_reference_line_smoke EXCLUDE_FROM_ALL
    test_reference_line_smoke.cpp)
target_link_libraries(test_reference_line_smoke PRIVATE
    reference_line)
