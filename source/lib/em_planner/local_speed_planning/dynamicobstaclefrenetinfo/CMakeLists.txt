add_library(dynamicobsfrenet STATIC dynamicobsfrenet.cpp)
target_link_libraries(dynamicobsfrenet PUBLIC pathmatcher
    localpathreferline)