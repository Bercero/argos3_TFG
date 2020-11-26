/**
 * @file <argos3/plugins/robots/foot-bot/control_interface/ci_kilobot_custom_communication_sensor.cpp>
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 */

#include "ci_kilobot_custom_communication_sensor.h"

#ifdef ARGOS_WITH_LUA
#include <argos3/core/wrappers/lua/lua_utility.h>
#endif

namespace argos {

   /****************************************/
   /****************************************/

   CCI_KilobotCustomCommunicationSensor::CCI_KilobotCustomCommunicationSensor() :
      m_bMessageSent(false) {
   }

   /****************************************/
   /****************************************/

   const CCI_KilobotCustomCommunicationSensor::TPackets& CCI_KilobotCustomCommunicationSensor::GetPackets() const {
      return m_tPackets;
   }

   /****************************************/
   /****************************************/

   bool CCI_KilobotCustomCommunicationSensor::MessageSent() const {
      return m_bMessageSent;
   }

   /****************************************/
   /****************************************/

#ifdef ARGOS_WITH_LUA
   void CCI_KilobotCustomCommunicationSensor::CreateLuaState(lua_State* pt_lua_state) {
      // TODO
   }
#endif

   /****************************************/
   /****************************************/

#ifdef ARGOS_WITH_LUA
   void CCI_KilobotCustomCommunicationSensor::ReadingsToLuaState(lua_State* pt_lua_state) {
      // TODO
   }
#endif


   /****************************************/
   /****************************************/

}
