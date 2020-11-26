/**
 * @file <argos3/plugins/robots/kilobot_custom/simulator/kilobot_custom_led_default_actuator.h>
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 */

#ifndef KILOBOT_CUSTOM_LED_DEFAULT_ACTUATOR_H
#define KILOBOT_CUSTOM_LED_DEFAULT_ACTUATOR_H

#include <string>
#include <map>

namespace argos {
   class CKilobotCustomLEDsDefaultActuator;
}

#include <argos3/plugins/robots/kilobot_custom/control_interface/ci_kilobot_custom_led_actuator.h>
#include <argos3/plugins/simulator/entities/led_equipped_entity.h>
#include <argos3/core/simulator/actuator.h>

namespace argos {

   class CKilobotCustomLEDDefaultActuator : public CSimulatedActuator,
                                      public CCI_KilobotCustomLEDActuator {

   public:

      CKilobotCustomLEDDefaultActuator();

      virtual ~CKilobotCustomLEDDefaultActuator() {}

      virtual void SetRobot(CComposableEntity& c_entity);

      virtual void Init(TConfigurationNode& t_tree);
      virtual void Update();
      virtual void Reset();
      virtual void Destroy();

   private:

      CLEDEquippedEntity* m_pcLEDEquippedEntity;

   };

}

#endif
