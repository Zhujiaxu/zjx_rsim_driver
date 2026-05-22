add_library(traffic_rule_handler STATIC
    TrafficRuleHandler.cpp)

target_include_directories(traffic_rule_handler PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${RSIM_PLUGIN_SDK_INCLUDE})

target_link_libraries(traffic_rule_handler PUBLIC
    driver_common
    map_helper)
