add_library(footbot_lamb MODULE footbot_lamb.h footbot_lamb.cpp)
target_link_libraries(footbot_lamb
    argos3core_simulator
    argos3plugin_simulator_footbot
    argos3plugin_simulator_genericrobot)
