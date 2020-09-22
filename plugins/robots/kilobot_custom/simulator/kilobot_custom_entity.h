/**
 * @file <argos3/plugins/robots/kilobot/simulator/kilobot_custom_entity.h>
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 * @author Vito Trianni - <vito.trianni@istc.cnr.it>
 */

#ifndef KILOBOT_CUSTOM_ENTITY_H
#define KILOBOT_CUSTOM_ENTITY_H

namespace argos {
   class CControllableEntity;
   class CEmbodiedEntity;
   class CKilobotCustomEntity;
   class CLEDEquippedEntity;
   class CLightSensorEquippedEntity;
   class CKilobotCommunicationEntity;
   class CGroundSensorEquippedEntity;
}

#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/plugins/simulator/entities/wheeled_entity.h>
#include <argos3/core/utility/datatypes/color.h>


namespace argos {

   class CKilobotCustomEntity : public CComposableEntity {

   public:

      ENABLE_VTABLE();

   public:

      CKilobotCustomEntity();

      CKilobotCustomEntity(const std::string& str_id,
                     const std::string& str_controller_id,
                     const CVector3& c_position = CVector3(),
                     const CQuaternion& c_orientation = CQuaternion(),
                     Real f_communication_range = 0.1f);

      virtual void Init(TConfigurationNode& t_tree);
      virtual void Reset();
      virtual void Destroy();

      virtual void UpdateComponents();

      inline CControllableEntity& GetControllableEntity() {
         return *m_pcControllableEntity;
      }

      inline CEmbodiedEntity& GetEmbodiedEntity() {
         return *m_pcEmbodiedEntity;
      }

      inline CLEDEquippedEntity& GetLEDEquippedEntity() {
         return *m_pcLEDEquippedEntity;
      }

      inline CLightSensorEquippedEntity& GetLightSensorEquippedEntity() {
         return *m_pcLightSensorEquippedEntity;
      }

      inline CKilobotCommunicationEntity& GetKilobotCommunicationEntity() {
         return *m_pcKilobotCommunicationEntity;
      }

      inline CWheeledEntity& GetWheeledEntity() {
         return *m_pcWheeledEntity;
      }

      inline CGroundSensorEquippedEntity& GetGroundSensorEquippedEntity() {
         return *m_pcGroundSensorEquippedEntity;
      }

      // inline CColor GetColor() {
      //    return main_color;
      // }

      virtual std::string GetTypeDescription() const {
         return "kilobot"; //TODO "kilobot_custom"
      }

   private:
     // CColor main_color;

    CControllableEntity*         m_pcControllableEntity;
    CEmbodiedEntity*             m_pcEmbodiedEntity;
    CLEDEquippedEntity*          m_pcLEDEquippedEntity;
    CLightSensorEquippedEntity*  m_pcLightSensorEquippedEntity;
    CKilobotCommunicationEntity* m_pcKilobotCommunicationEntity;
    CWheeledEntity*              m_pcWheeledEntity;
    CGroundSensorEquippedEntity* m_pcGroundSensorEquippedEntity;

   };

}

#endif
