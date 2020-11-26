/**
 * @file <argos3/plugins/robots/kilobot_custom/control_interface/kilobot_custom_communication_default_sensor.h>
 *
 * @brief This file provides the definition of the kilobot_custom communicationdefault sensor.
 *
 * This file provides the definition of the kilobot_custom communicationdefault sensor.
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 */

#ifndef KILOBOT_CUSTOM_COMMUNICATION_DEFAULT_SENSOR_H
#define KILOBOT_CUSTOM_COMMUNICATION_DEFAULT_SENSOR_H

namespace argos {
   class CKilobotCustomCommunicationDefaultSensor;
   class CKilobotCustomCommunicationEntity;
   class CKilobotCustomCommunicationMedium;
   class CKilobotCustomEntity;
   class CControllableEntity;
}

#include <argos3/core/simulator/sensor.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/plugins/robots/kilobot_custom/control_interface/ci_kilobot_custom_communication_sensor.h>

namespace argos {

   class CKilobotCustomCommunicationDefaultSensor : public CCI_KilobotCustomCommunicationSensor,
                                              public CSimulatedSensor {

   public:

      CKilobotCustomCommunicationDefaultSensor();
      virtual ~CKilobotCustomCommunicationDefaultSensor() {}

      virtual void SetRobot(CComposableEntity& c_entity);
      virtual void Init(TConfigurationNode& t_tree);
      virtual void Update();
      virtual void Reset();
      virtual void Destroy();

   private:

      CKilobotCustomEntity*              m_pcRobot;
      CKilobotCustomCommunicationEntity* m_pcCommEntity;
      CKilobotCustomCommunicationMedium* m_pcMedium;
      CControllableEntity*         m_pcControllableEntity;
      Real                         m_fDistanceNoiseStdDev;
      CRandom::CRNG*               m_pcRNG;
      CSpace&                      m_cSpace;
      bool                         m_bShowRays;
   };

}

#endif
