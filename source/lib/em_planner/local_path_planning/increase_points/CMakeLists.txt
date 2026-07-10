add_library(increasepoints STATIC increasepoints.cpp)
target_link_libraries(increasepoints PUBLIC
    dynamic_programming
    quintic_polynomial)
target_include_directories(increasepoints PUBLIC
    ${CMAKE_CURRENT_LIST_DIR})