add_library(reference_line STATIC
    ReferenceLineProvider.cpp)

target_include_directories(reference_line PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(reference_line PUBLIC
    map_helper)
