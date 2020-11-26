/**
 * @file <argos3/plugins/robots/kilobot_custom/control_interface/ci_kilobot_custom_communication_actuator.h>
 *
 * @brief This file provides the definition of the kilobot_custom communication actuator.
 *
 * This file provides the definition of the kilobot_custom communication actuator.
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 */

#ifndef CCI_KILOBOT_CUSTOM_COMMUNICATION_ACTUATOR_H
#define CCI_KILOBOT_CUSTOM_COMMUNICATION_ACTUATOR_H

namespace argos {
   class CCI_KilobotCustomCommunicationActuator;
}

#include <argos3/core/control_interface/ci_actuator.h>
#include <argos3/plugins/robots/kilobot_custom/control_interface/kilolib.h>
#include <argos3/core/utility/datatypes/datatypes.h>
#include <vector>

namespace argos {

   class CCI_KilobotCustomCommunicationActuator : public CCI_Actuator {

   public:

      CCI_KilobotCustomCommunicationActuator();
      virtual ~CCI_KilobotCustomCommunicationActuator() {}

      virtual void SetMessage(message_t* pt_msg);

#ifdef ARGOS_WITH_LUA
      virtual void CreateLuaState(lua_State* pt_lua_state);
#endif

   protected:

      message_t* m_ptMessage;

   };

}

#endif
