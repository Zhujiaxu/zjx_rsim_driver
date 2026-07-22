add_library(planning_start SHARED
    PlanningStartPoint.cpp)

if(NOT DEFINED RSIM_PLUGIN_SDK_INCLUDE)
    set(RSIM_PLUGIN_SDK_INCLUDE
        ${CMAKE_CURRENT_LIST_DIR}/../../../../third_party/RSimAPI/source)
endif()

target_include_directories(planning_start PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    ${RSIM_PLUGIN_SDK_INCLUDE})

target_compile_features(planning_start PUBLIC cxx_std_17)

target_link_libraries(planning_start PUBLIC
    cartesian_to_frenet)


    add_executable(test_planning_start_point EXCLUDE_FROM_ALL
        test_planning_start_point.cpp)
    target_link_libraries(test_planning_start_point PRIVATE planning_start)

