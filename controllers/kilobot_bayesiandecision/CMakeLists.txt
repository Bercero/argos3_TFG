add_library(kilobot_bayesiandecision SHARED kilobot_bayesiandecision.h kilobot_bayesiandecision.cpp)
find_package(Boost REQUIRED)
target_link_libraries(kilobot_bayesiandecision
    argos3core_simulator
    argos3plugin_simulator_kilobot_custom
    argos3plugin_simulator_kilolib #TODO sobrescribe la original
    argos3plugin_simulator_genericrobot
    Boost::boost)
