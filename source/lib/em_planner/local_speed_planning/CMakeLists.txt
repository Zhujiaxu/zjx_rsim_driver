add_library(local_speed_planning INTERFACE)

target_include_directories(local_speed_planning INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/..)

target_compile_features(local_speed_planning INTERFACE cxx_std_17)
