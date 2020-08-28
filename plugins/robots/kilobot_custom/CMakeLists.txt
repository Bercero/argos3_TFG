#
# kilobot headers
#
# argos3/plugins/robots/kilobot/control_interface
set(ARGOS3_HEADERS_PLUGINS_ROBOTS_KILOBOT_CONTROLINTERFACE
  control_interface/ci_kilobot_communication_actuator.h
  control_interface/ci_kilobot_communication_sensor.h
  control_interface/ci_kilobot_controller.h
  control_interface/ci_kilobot_led_actuator.h
  control_interface/ci_kilobot_light_sensor.h
  control_interface/kilolib.h
  control_interface/debug.h
  control_interface/message.h
  control_interface/message_crc.h)
# argos3/plugins/robots/kilobot/simulator
if(ARGOS_BUILD_FOR_SIMULATOR)
  set(ARGOS3_HEADERS_PLUGINS_ROBOTS_KILOBOT_SIMULATOR
    simulator/dynamics2d_kilobot_model.h
    simulator/pointmass3d_kilobot_model.h
    simulator/kilobot_entity.h
    simulator/kilobot_measures.h
    simulator/kilobot_led_default_actuator.h
    simulator/kilobot_light_rotzonly_sensor.h
    simulator/kilobot_communication_default_actuator.h
    simulator/kilobot_communication_default_sensor.h
    simulator/kilobot_communication_entity.h
    simulator/kilobot_communication_medium.h)
endif(ARGOS_BUILD_FOR_SIMULATOR)

#
# kilobot sources
#
set(ARGOS3_SOURCES_PLUGINS_ROBOTS_KILOBOT
  ${ARGOS3_HEADERS_PLUGINS_ROBOTS_KILOBOT_CONTROLINTERFACE}
  control_interface/ci_kilobot_communication_actuator.cpp
  control_interface/ci_kilobot_communication_sensor.cpp
  control_interface/ci_kilobot_controller.cpp
  control_interface/ci_kilobot_led_actuator.cpp
  control_interface/ci_kilobot_light_sensor.cpp)

if(ARGOS_BUILD_FOR_SIMULATOR)
  set(ARGOS3_SOURCES_PLUGINS_ROBOTS_KILOBOT
    ${ARGOS3_SOURCES_PLUGINS_ROBOTS_KILOBOT}
    ${ARGOS3_HEADERS_PLUGINS_ROBOTS_KILOBOT_SIMULATOR}
    simulator/dynamics2d_kilobot_model.cpp
    simulator/pointmass3d_kilobot_model.cpp
    simulator/kilobot_entity.cpp
    simulator/kilobot_led_default_actuator.cpp
    simulator/kilobot_light_rotzonly_sensor.cpp
    simulator/kilobot_communication_default_actuator.cpp
    simulator/kilobot_communication_default_sensor.cpp
    simulator/kilobot_communication_entity.cpp
    simulator/kilobot_communication_medium.cpp)
  # Compile the graphical visualization only if the necessary libraries have been found
  include(ARGoSCheckQTOpenGL)
  if(ARGOS_COMPILE_QTOPENGL)
    set(ARGOS3_HEADERS_PLUGINS_ROBOTS_KILOBOT_SIMULATOR
      ${ARGOS3_HEADERS_PLUGINS_ROBOTS_KILOBOT_SIMULATOR}
      simulator/qtopengl_kilobot.h)
    set(ARGOS3_SOURCES_PLUGINS_ROBOTS_KILOBOT
      ${ARGOS3_SOURCES_PLUGINS_ROBOTS_KILOBOT}
      simulator/qtopengl_kilobot.h
      simulator/qtopengl_kilobot.cpp)
  endif(ARGOS_COMPILE_QTOPENGL)
endif(ARGOS_BUILD_FOR_SIMULATOR)

#
# Create kilobot plugin
#
add_library(argos3plugin_${ARGOS_BUILD_FOR}_kilobot_custom SHARED ${ARGOS3_SOURCES_PLUGINS_ROBOTS_KILOBOT})
target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_kilobot_custom
  argos3plugin_${ARGOS_BUILD_FOR}_genericrobot
  argos3plugin_${ARGOS_BUILD_FOR}_dynamics2d
  argos3plugin_${ARGOS_BUILD_FOR}_pointmass3d)
if(ARGOS_COMPILE_QTOPENGL)
  target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_kilobot_custom argos3plugin_${ARGOS_BUILD_FOR}_qtopengl)
endif(ARGOS_COMPILE_QTOPENGL)

#
# Create kilolib
#
if(ARGOS_BUILD_FOR_SIMULATOR)
  add_library(argos3plugin_simulator_kilolib
    control_interface/kilolib.c
    control_interface/message_crc.c)
  if(RT_FOUND)
    target_link_libraries(argos3plugin_simulator_kilolib ${RT_LIBRARIES})
  endif(RT_FOUND)
endif(ARGOS_BUILD_FOR_SIMULATOR)
