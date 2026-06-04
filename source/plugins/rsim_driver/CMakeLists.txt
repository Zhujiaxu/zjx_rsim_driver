add_library(rsim_driver SHARED
    RSimDriverPlugin.cpp)

target_include_directories(rsim_driver PRIVATE
    ${RSIM_PLUGIN_SDK_INCLUDE})

target_link_libraries(rsim_driver PRIVATE
    map_helper
    reference_line
    pugixml::pugixml)

set_target_properties(rsim_driver PROPERTIES
    PREFIX "lib"
    SUFFIX ".so")
