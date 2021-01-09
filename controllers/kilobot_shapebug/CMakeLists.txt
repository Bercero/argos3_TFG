add_library(kilobot_shapebug SHARED kilobot_shapebug.h kilobot_shapebug.cpp)
target_link_libraries(kilobot_shapebug
    argos3core_simulator
    argos3plugin_simulator_kilobot_custom
    argos3plugin_simulator_kilolib
    argos3plugin_simulator_genericrobot)
