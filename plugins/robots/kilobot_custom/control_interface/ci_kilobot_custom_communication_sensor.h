/**
 * @file <argos3/plugins/robots/kilobot_custom/control_interface/ci_kilobot_custom_communication_sensor.h>
 *
 * @brief This file provides the definition of the kilobot_custom communication sensor.
 *
 * This file provides the definition of the kilobot_custom communication sensor.
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 */

#ifndef CCI_KILOBOT_CUSTOM_COMMUNICATION_SENSOR_H
#define CCI_KILOBOT_CUSTOM_COMMUNICATION_SENSOR_H

namespace argos {
   class CCI_KilobotCustomCommunicationSensor;
}

#include <argos3/core/control_interface/ci_sensor.h>
#include <argos3/plugins/robots/kilobot_custom/control_interface/kilolib.h>
#include <argos3/core/utility/datatypes/datatypes.h>
#include <vector>

namespace argos {

   class CCI_KilobotCustomCommunicationSensor : public CCI_Sensor {

   public:

      struct SPacket {
         const message_t* Message;
         distance_measurement_t Distance;
      };

      typedef std::vector<SPacket> TPackets;

   public:

      CCI_KilobotCustomCommunicationSensor();
      virtual ~CCI_KilobotCustomCommunicationSensor() {}

      const TPackets& GetPackets() const;

      bool MessageSent() const;

#ifdef ARGOS_WITH_LUA
      virtual void CreateLuaState(lua_State* pt_lua_state);

      virtual void ReadingsToLuaState(lua_State* pt_lua_state);
#endif

   protected:

      TPackets m_tPackets;
      bool m_bMessageSent;

   };

}

#endif
