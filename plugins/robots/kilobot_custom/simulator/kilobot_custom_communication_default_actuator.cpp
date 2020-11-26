#include "kilobot_custom_communication_default_actuator.h"
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/core/simulator/simulator.h>

namespace argos {

   /****************************************/
   /****************************************/

   void CKilobotCustomCommunicationDefaultActuator::SetRobot(CComposableEntity& c_entity) {
      m_pcCommEntity = &c_entity.GetComponent<CKilobotCustomCommunicationEntity>("kilocomm");
      m_pcCommEntity->Enable();
   }

   /****************************************/
   /****************************************/

   void CKilobotCustomCommunicationDefaultActuator::Update() {
      m_pcCommEntity->SetTxMessage(m_ptMessage);
   }

   /****************************************/
   /****************************************/

   void CKilobotCustomCommunicationDefaultActuator::Reset() {
   }

   /****************************************/
   /****************************************/

   void CKilobotCustomCommunicationDefaultActuator::SetMessage(message_t* pt_msg) {
      CCI_KilobotCustomCommunicationActuator::SetMessage(pt_msg);
      m_pcCommEntity->SetTxStatus(CKilobotCustomCommunicationEntity::TX_ATTEMPT);
   }

   /****************************************/
   /****************************************/

   REGISTER_ACTUATOR(CKilobotCustomCommunicationDefaultActuator,
                     "kilobot_custom_communication", "default",
                     "Carlo Pinciroli [ilpincy@gmail.com]",
                     "1.0",
                     "The KilobotCustom communication actuator.",
                     "This actuator allows KilobotCustoms to perform situated communication, i.e., a form\n"
                     "of wireless communication whereby the receiver also knows the distance of the\n"
                     "sender.\n"
                     "To use this actuator, in controllers you must include the\n"
                     "ci_kilobot_custom_communication_actuator.h header.\n"
                     "This actuator only allows a KilobotCustom to send messages. To receive messages, you\n"
                     "need the range-and-bearing sensor.\n\n"
                     "REQUIRED XML CONFIGURATION\n\n"
                     "  <controllers>\n"
                     "    ...\n"
                     "    <my_controller ...>\n"
                     "      ...\n"
                     "      <actuators>\n"
                     "        ...\n"
                     "        <kilobot_custom_communication implementation=\"default\" />\n"
                     "        ...\n"
                     "      </actuators>\n"
                     "      ...\n"
                     "    </my_controller>\n"
                     "    ...\n"
                     "  </controllers>\n\n"
                     "OPTIONAL XML CONFIGURATION\n\n"
                     "None for the time being.\n",
                     "Usable");
   
}
