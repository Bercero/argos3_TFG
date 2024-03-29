/**
 * @file <argos3/plugins/robots/kilobot_custom/control_interface/ci_kilobot_custom_light_sensor.h>
 *
 * @brief This file provides the definition of the kilobot_custom light sensor.
 *
 * This file provides the definition of the kilobot_custom light sensor.
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 * @author Vito Trianni - <vito.trianni@istc.cnr.it>
 */

#ifndef CCI_KILOBOT_CUSTOM_LIGHT_SENSOR_H
#define CCI_KILOBOT_CUSTOM_LIGHT_SENSOR_H

namespace argos {
   class CCI_KilobotCustomLightSensor;
}

#include <argos3/core/control_interface/ci_sensor.h>
#include <argos3/core/utility/math/angles.h>
#include <vector>

namespace argos {

   class CCI_KilobotCustomLightSensor : public CCI_Sensor {

   public:

      CCI_KilobotCustomLightSensor();
      virtual ~CCI_KilobotCustomLightSensor() {}

      /**
       * Returns the readings of this sensor
       */
      inline SInt16 GetReading() const {
         return m_nReading;
      }

#ifdef ARGOS_WITH_LUA
      virtual void CreateLuaState(lua_State* pt_lua_state);

      virtual void ReadingsToLuaState(lua_State* pt_lua_state);
#endif

   protected:

      SInt16 m_nReading;
   };

}

#endif
