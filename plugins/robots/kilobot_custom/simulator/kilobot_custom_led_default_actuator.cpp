/**
 * @file <argos3/plugins/robots/kilobot_custom/simulator/kilobot_custom_led_default_actuator.cpp>
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 */

#include "kilobot_custom_led_default_actuator.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/logging/argos_log.h>

namespace argos {

   /****************************************/
   /****************************************/

   CKilobotCustomLEDDefaultActuator::CKilobotCustomLEDDefaultActuator() :
      m_pcLEDEquippedEntity(NULL) {}

   /****************************************/
   /****************************************/

   void CKilobotCustomLEDDefaultActuator::SetRobot(CComposableEntity& c_entity) {
      m_pcLEDEquippedEntity = &(c_entity.GetComponent<CLEDEquippedEntity>("leds"));
   }

   /****************************************/
   /****************************************/

   void CKilobotCustomLEDDefaultActuator::Init(TConfigurationNode& t_tree) {
      try {
         CCI_KilobotCustomLEDActuator::Init(t_tree);
         m_pcLEDEquippedEntity->Enable();
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Error initializing the LEDs default actuator", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CKilobotCustomLEDDefaultActuator::Update() {
      m_pcLEDEquippedEntity->SetLEDColor(0, m_cColor);
   }

   /****************************************/
   /****************************************/

   void CKilobotCustomLEDDefaultActuator::Reset() {
      SetColor(CColor::BLACK);
   }

   /****************************************/
   /****************************************/

   void CKilobotCustomLEDDefaultActuator::Destroy() {
      m_pcLEDEquippedEntity->Disable();
   }

   /****************************************/
   /****************************************/

}

REGISTER_ACTUATOR(CKilobotCustomLEDDefaultActuator,
                  "kilobot_custom_led", "default",
                  "Carlo Pinciroli [ilpincy@gmail.com]",
                  "1.0",
                  "The KilobotCustom LED actuator.",
                  "This actuator controls the LED of the kilobot_custom. For a complete description of its\n"
                  "usage, refer to the ci_kilobot_custom_led_actuator.h file.\n\n"
                  "REQUIRED XML CONFIGURATION\n\n"
                  "  <controllers>\n"
                  "    ...\n"
                  "    <my_controller ...>\n"
                  "      ...\n"
                  "      <actuators>\n"
                  "        ...\n"
                  "        <kilobot_custom-led implementation=\"default\" />\n"
                  "        ...\n"
                  "      </actuators>\n"
                  "      ...\n"
                  "    </my_controller>\n"
                  "    ...\n"
                  "  </controllers>\n\n"
                  "OPTIONAL XML CONFIGURATION\n\n"
                  "None.\n",
                  "Usable"
   );

