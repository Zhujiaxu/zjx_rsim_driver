add_library(default_driver SHARED
    DefaultDriverController.cpp)

target_include_directories(default_driver PRIVATE ${RSIM_PLUGIN_SDK_INCLUDE})

set_target_properties(default_driver PROPERTIES
    PREFIX "lib"
    SUFFIX ".so")
