add_library(kilobot_formation SHARED kilobot_formation.h kilobot_formation.cpp)
target_link_libraries(kilobot_formation
    argos3core_simulator
    argos3plugin_simulator_kilobot
    argos3plugin_simulator_kilolib
    argos3plugin_simulator_genericrobot)
