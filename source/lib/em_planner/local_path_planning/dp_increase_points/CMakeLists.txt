add_library(dpincreasepoints STATIC dpincreasepoints.cpp)

target_include_directories(dpincreasepoints PUBLIC
    ${CMAKE_CURRENT_LIST_DIR})
    
target_link_libraries(dpincreasepoints PUBLIC
    dpplanner
    quintic_polynomial)
