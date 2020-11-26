/**
 * @file <argos3/plugins/robots/kilobot_custom/control_interface/ci_kilobot_custom_led_actuator.h>
 *
 * @author Carlo Pinciroli <ilpincy@gmail.com>
 */

#ifndef CCI_KILOBOT_CUSTOM_LED_ACTUATOR_H
#define CCI_KILOBOT_CUSTOM_LED_ACTUATOR_H

namespace argos {
   class CCI_KilobotCustomLedActuator;
}

#include <argos3/core/control_interface/ci_actuator.h>
#include <argos3/core/utility/datatypes/color.h>

namespace argos {

   class CCI_KilobotCustomLEDActuator : public CCI_Actuator {

   public:

      CCI_KilobotCustomLEDActuator() {}

      virtual ~CCI_KilobotCustomLEDActuator() {}

      /**
       * @brief Sets the color of the LED.
       */
      virtual void SetColor(const CColor& c_color);
      
#ifdef ARGOS_WITH_LUA
      virtual void CreateLuaState(lua_State* pt_lua_state) {}
#endif

   protected:

      CColor m_cColor;

   };

}

#endif
