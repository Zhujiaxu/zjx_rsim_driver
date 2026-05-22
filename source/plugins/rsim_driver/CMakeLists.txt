add_library(rsim_driver SHARED
    RSimDriverPlugin.cpp)

target_include_directories(rsim_driver PRIVATE
    ${RSIM_PLUGIN_SDK_INCLUDE})

target_link_libraries(rsim_driver PRIVATE
    driver_common
    perception_helper
    sampling_planner
    em_planner
    traffic_rule_handler
    map_helper
    rsim-client
    pugixml::pugixml)

set_target_properties(rsim_driver PROPERTIES
    PREFIX "lib"
    SUFFIX ".so")
