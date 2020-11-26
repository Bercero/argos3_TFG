/**
 * @file <argos3/plugins/robots/foot-bot/control_interface/ci_kilobot_custom_communication_actuator.cpp>
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 */

#include "ci_kilobot_custom_communication_actuator.h"

#ifdef ARGOS_WITH_LUA
#include <argos3/core/wrappers/lua/lua_utility.h>
#endif

namespace argos {

   /****************************************/
   /****************************************/

   CCI_KilobotCustomCommunicationActuator::CCI_KilobotCustomCommunicationActuator() {
   }

   /****************************************/
   /****************************************/

   void CCI_KilobotCustomCommunicationActuator::SetMessage(message_t* pt_msg) {
      m_ptMessage = pt_msg;
   }

   /****************************************/
   /****************************************/

#ifdef ARGOS_WITH_LUA
   void CCI_KilobotCustomCommunicationActuator::CreateLuaState(lua_State* pt_lua_state) {
      // TODO
   }
#endif

   /****************************************/
   /****************************************/

}
