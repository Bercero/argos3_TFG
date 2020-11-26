#
# kilobot_custom headers
#
# argos3/plugins/robots/kilobot_custom/control_interface
set(ARGOS3_HEADERS_PLUGINS_ROBOTS_KILOBOT_CUSTOM_CONTROLINTERFACE
  control_interface/ci_kilobot_custom_communication_actuator.h
  control_interface/ci_kilobot_custom_communication_sensor.h
  control_interface/ci_kilobot_custom_controller.h
  control_interface/ci_kilobot_custom_led_actuator.h
  control_interface/ci_kilobot_custom_light_sensor.h
  control_interface/kilolib.h
  control_interface/debug.h
  control_interface/message.h
  control_interface/message_crc.h)
# argos3/plugins/robots/kilobot_custom/simulator
if(ARGOS_BUILD_FOR_SIMULATOR)
  set(ARGOS3_HEADERS_PLUGINS_ROBOTS_KILOBOT_CUSTOM_SIMULATOR
    simulator/dynamics2d_kilobot_custom_model.h
    simulator/pointmass3d_kilobot_custom_model.h
    simulator/kilobot_custom_entity.h
    simulator/kilobot_custom_measures.h
    simulator/kilobot_custom_led_default_actuator.h
    simulator/kilobot_custom_light_rotzonly_sensor.h
    simulator/kilobot_custom_communication_default_actuator.h
    simulator/kilobot_custom_communication_default_sensor.h
    simulator/kilobot_custom_communication_entity.h
    simulator/kilobot_custom_communication_medium.h)
endif(ARGOS_BUILD_FOR_SIMULATOR)

#
# kilobot_custom sources
#
set(ARGOS3_SOURCES_PLUGINS_ROBOTS_KILOBOT_CUSTOM
  ${ARGOS3_HEADERS_PLUGINS_ROBOTS_KILOBOT_CUSTOM_CONTROLINTERFACE}
  control_interface/ci_kilobot_custom_communication_actuator.cpp
  control_interface/ci_kilobot_custom_communication_sensor.cpp
  control_interface/ci_kilobot_custom_controller.cpp
  control_interface/ci_kilobot_custom_led_actuator.cpp
  control_interface/ci_kilobot_custom_light_sensor.cpp)

if(ARGOS_BUILD_FOR_SIMULATOR)
  set(ARGOS3_SOURCES_PLUGINS_ROBOTS_KILOBOT_CUSTOM
    ${ARGOS3_SOURCES_PLUGINS_ROBOTS_KILOBOT_CUSTOM}
    ${ARGOS3_HEADERS_PLUGINS_ROBOTS_KILOBOT_CUSTOM_SIMULATOR}
    simulator/dynamics2d_kilobot_custom_model.cpp
    simulator/pointmass3d_kilobot_custom_model.cpp
    simulator/kilobot_custom_entity.cpp
    simulator/kilobot_custom_led_default_actuator.cpp
    simulator/kilobot_custom_light_rotzonly_sensor.cpp
    simulator/kilobot_custom_communication_default_actuator.cpp
    simulator/kilobot_custom_communication_default_sensor.cpp
    simulator/kilobot_custom_communication_entity.cpp
    simulator/kilobot_custom_communication_medium.cpp)
  # Compile the graphical visualization only if the necessary libraries have been found
  include(ARGoSCheckQTOpenGL)
  if(ARGOS_COMPILE_QTOPENGL)
    set(ARGOS3_HEADERS_PLUGINS_ROBOTS_KILOBOT_CUSTOM_SIMULATOR
      ${ARGOS3_HEADERS_PLUGINS_ROBOTS_KILOBOT_CUSTOM_SIMULATOR}
      simulator/qtopengl_kilobot_custom.h)
    set(ARGOS3_SOURCES_PLUGINS_ROBOTS_KILOBOT_CUSTOM
      ${ARGOS3_SOURCES_PLUGINS_ROBOTS_KILOBOT_CUSTOM}
      simulator/qtopengl_kilobot_custom.h
      simulator/qtopengl_kilobot_custom.cpp)
  endif(ARGOS_COMPILE_QTOPENGL)
endif(ARGOS_BUILD_FOR_SIMULATOR)

#
# Create kilobot_custom plugin
#
add_library(argos3plugin_${ARGOS_BUILD_FOR}_kilobot_custom SHARED ${ARGOS3_SOURCES_PLUGINS_ROBOTS_KILOBOT_CUSTOM})
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

#
# Installation
#
install(FILES ${ARGOS3_HEADERS_PLUGINS_ROBOTS_KILOBOT_CUSTOM_CONTROLINTERFACE} DESTINATION include/argos3/plugins/robots/kilobot_custom/control_interface)

if(ARGOS_BUILD_FOR_SIMULATOR)
  install(FILES ${ARGOS3_HEADERS_PLUGINS_ROBOTS_KILOBOT_CUSTOM_SIMULATOR}      DESTINATION include/argos3/plugins/robots/kilobot_custom/simulator)
endif(ARGOS_BUILD_FOR_SIMULATOR)

install(TARGETS argos3plugin_${ARGOS_BUILD_FOR}_kilobot_custom
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib/argos3
  ARCHIVE DESTINATION lib/argos3)

if(ARGOS_BUILD_FOR_SIMULATOR)
  install(TARGETS argos3plugin_simulator_kilolib
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib/argos3
    ARCHIVE DESTINATION lib/argos3)
endif(ARGOS_BUILD_FOR_SIMULATOR)
