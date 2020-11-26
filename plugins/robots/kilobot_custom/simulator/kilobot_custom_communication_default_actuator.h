/**
 * @file <argos3/plugins/robots/kilobot_custom/simulator/kilobot_custom_communication_default_actuator.h>
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 */

#ifndef RANGE_AND_BEARING_DEFAULT_ACTUATOR_H
#define RANGE_AND_BEARING_DEFAULT_ACTUATOR_H

#include <string>
#include <map>

namespace argos {
   class CKilobotCustomCommunicationDefaultActuator;
}

#include <argos3/core/simulator/actuator.h>
#include <argos3/plugins/robots/kilobot_custom/control_interface/ci_kilobot_custom_communication_actuator.h>
#include <argos3/plugins/robots/kilobot_custom/simulator/kilobot_custom_communication_entity.h>

namespace argos {

   class CKilobotCustomCommunicationDefaultActuator : public CSimulatedActuator,
                                                public CCI_KilobotCustomCommunicationActuator {

   public:

      CKilobotCustomCommunicationDefaultActuator() {}
      virtual ~CKilobotCustomCommunicationDefaultActuator() {}
      virtual void SetRobot(CComposableEntity& c_entity);
      virtual void Update();
      virtual void Reset();
      virtual void SetMessage(message_t* pt_msg);

   private:

      CKilobotCustomCommunicationEntity* m_pcCommEntity;

   };

}

#endif
