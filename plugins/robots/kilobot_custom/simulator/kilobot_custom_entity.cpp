/**
 * @file <argos3/plugins/robots/kilobot_custom/simulator/kilobot_custom_entity.cpp>
 *TODO
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 */

#include "kilobot_custom_entity.h"
#include "kilobot_custom_measures.h"

#include <argos3/core/utility/math/matrix/rotationmatrix3.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/entity/controllable_entity.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/plugins/simulator/entities/led_equipped_entity.h>
#include <argos3/plugins/simulator/entities/light_sensor_equipped_entity.h>
#include <argos3/plugins/simulator/entities/ground_sensor_equipped_entity.h>
#include <argos3/plugins/robots/kilobot_custom/simulator/kilobot_custom_communication_entity.h>

namespace argos {

   /****************************************/
   /****************************************/

   CKilobotCustomEntity::CKilobotCustomEntity() :
      // main_color(0,0,255),
      CComposableEntity(NULL),
      m_pcControllableEntity(NULL),
      m_pcEmbodiedEntity(NULL),
      m_pcLEDEquippedEntity(NULL),
      m_pcLightSensorEquippedEntity(NULL),
      m_pcKilobotCustomCommunicationEntity(NULL),
      m_pcWheeledEntity(NULL),
      m_pcGroundSensorEquippedEntity(NULL){
   }

   /****************************************/
   /****************************************/

   CKilobotCustomEntity::CKilobotCustomEntity(const std::string& str_id,
                                  const std::string& str_controller_id,
                                  const CVector3& c_position,
                                  const CQuaternion& c_orientation,
                                  Real f_communication_range) :
      // main_color(0,0,255),
      CComposableEntity(NULL, str_id),
      m_pcControllableEntity(NULL),
      m_pcEmbodiedEntity(NULL),
      m_pcLEDEquippedEntity(NULL),
      m_pcLightSensorEquippedEntity(NULL),
      m_pcWheeledEntity(NULL),
      m_pcGroundSensorEquippedEntity(NULL) {
      try {
         /*
          * Create and init components
          */
         /*
          * Embodied entity
          * Better to put this first, because many other entities need this one
          */
         m_pcEmbodiedEntity = new CEmbodiedEntity(this, "body_0", c_position, c_orientation);
         SAnchor& cLightAnchor = m_pcEmbodiedEntity->AddAnchor("light", KILOBOT_CUSTOM_LIGHT_SENSOR_OFFSET);
         SAnchor& cCommAnchor = m_pcEmbodiedEntity->AddAnchor("comm", CVector3(0.0, 0.0, KILOBOT_CUSTOM_RAB_ELEVATION));
         AddComponent(*m_pcEmbodiedEntity);
         /* Wheeled entity and wheel positions (left, right) */
         m_pcWheeledEntity = new CWheeledEntity(this, "wheels_0", 2);
         AddComponent(*m_pcWheeledEntity);
         m_pcWheeledEntity->SetWheel(0, CVector3(0.0f,  KILOBOT_CUSTOM_HALF_INTERPIN_DISTANCE, 0.0f), KILOBOT_CUSTOM_PIN_WHEEL_RADIUS);
         m_pcWheeledEntity->SetWheel(1, CVector3(0.0f, -KILOBOT_CUSTOM_HALF_INTERPIN_DISTANCE, 0.0f), KILOBOT_CUSTOM_PIN_WHEEL_RADIUS);
         /* LED equipped entity */
         m_pcLEDEquippedEntity = new CLEDEquippedEntity(this, "leds_0");
         AddComponent(*m_pcLEDEquippedEntity);
         m_pcLEDEquippedEntity->AddLED(CVector3(), m_pcEmbodiedEntity->GetOriginAnchor());
         /* Light sensor equipped entity */
         m_pcLightSensorEquippedEntity = new CLightSensorEquippedEntity(this, "light_0");
         AddComponent(*m_pcLightSensorEquippedEntity);
         m_pcLightSensorEquippedEntity->AddSensor(KILOBOT_CUSTOM_LIGHT_SENSOR_OFFSET,
                                                  CVector3(0.0f, 0.0f, 1.0f),
                                                  KILOBOT_CUSTOM_LIGHT_SENSOR_RANGE,
                                                  cLightAnchor);
         /* KilobotCustom communication entity */
         m_pcKilobotCustomCommunicationEntity =
            new CKilobotCustomCommunicationEntity(this,
                                            "kilocomm_0",
                                            9,
                                            f_communication_range,
                                            cCommAnchor,
                                            *m_pcEmbodiedEntity);
         AddComponent(*m_pcKilobotCustomCommunicationEntity);
         /* Ground sensor equipped entity */
         m_pcGroundSensorEquippedEntity =
            new CGroundSensorEquippedEntity(this, "ground_0");
         AddComponent(*m_pcGroundSensorEquippedEntity);
         //TODO usar valores de KILOBOT_GROUND_SENSOR_0_OFFSET
         m_pcGroundSensorEquippedEntity->AddSensor(CVector2(0.0, 0.0),
                                                   CGroundSensorEquippedEntity::TYPE_BLACK_WHITE,
                                                   m_pcEmbodiedEntity->GetOriginAnchor());
         m_pcGroundSensorEquippedEntity->AddSensor(CVector2(0.0, 0.01),
                                                   CGroundSensorEquippedEntity::TYPE_GRAYSCALE,
                                                   m_pcEmbodiedEntity->GetOriginAnchor());
         /* Controllable entity.  It must be the last one, for
            actuators/sensors to link to composing entities
            correctly */
         m_pcControllableEntity = new CControllableEntity(this, "controller_0");
         AddComponent(*m_pcControllableEntity);
         m_pcControllableEntity->SetController(str_controller_id);
         /* Update components */
         UpdateComponents();
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Failed to initialize entity \"" << GetId() << "\".", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CKilobotCustomEntity::Init(TConfigurationNode& t_tree) {
      try {
          // GetNodeAttributeOrDefault(t_tree,"color",main_color,main_color);
         /*
          * Init parent
          */
         CComposableEntity::Init(t_tree);
         /*
          * Create and init components
          */
         /* Embodied entity. Better to put this first, because many other entities need this one */
         m_pcEmbodiedEntity = new CEmbodiedEntity(this);
         AddComponent(*m_pcEmbodiedEntity);
         m_pcEmbodiedEntity->Init(GetNode(t_tree, "body"));
         /* Create an anchor for the light sensor - must be done after initializing the body */
         SAnchor& cLightAnchor = m_pcEmbodiedEntity->AddAnchor("light", KILOBOT_CUSTOM_LIGHT_SENSOR_OFFSET);
         SAnchor& cCommAnchor = m_pcEmbodiedEntity->AddAnchor("comm", CVector3(0.0, 0.0, KILOBOT_CUSTOM_RAB_ELEVATION));
         /* Wheeled entity and wheel positions (left, right) */
         m_pcWheeledEntity = new CWheeledEntity(this, "wheels_0", 2);
         AddComponent(*m_pcWheeledEntity);
         m_pcWheeledEntity->SetWheel(0, CVector3(0.0f,  KILOBOT_CUSTOM_HALF_INTERPIN_DISTANCE, 0.0f), KILOBOT_CUSTOM_PIN_WHEEL_RADIUS);
         m_pcWheeledEntity->SetWheel(1, CVector3(0.0f, -KILOBOT_CUSTOM_HALF_INTERPIN_DISTANCE, 0.0f), KILOBOT_CUSTOM_PIN_WHEEL_RADIUS);
         /* LED equipped entity */
         m_pcLEDEquippedEntity = new CLEDEquippedEntity(this, "leds_0");
         AddComponent(*m_pcLEDEquippedEntity);
         m_pcLEDEquippedEntity->AddLED(CVector3(),m_pcEmbodiedEntity->GetOriginAnchor());
         /* Light sensor equipped entity */
         m_pcLightSensorEquippedEntity = new CLightSensorEquippedEntity(this, "light_0");
         AddComponent(*m_pcLightSensorEquippedEntity);
         m_pcLightSensorEquippedEntity->AddSensor(KILOBOT_CUSTOM_LIGHT_SENSOR_OFFSET,
                                                  CVector3(0.0f,0.0f,1.0f),
                                                  KILOBOT_CUSTOM_LIGHT_SENSOR_RANGE,
                                                  cLightAnchor);
         /* KilobotCustom communication entity */
         Real fRange = 0.1f;
         GetNodeAttributeOrDefault(t_tree, "communication_range", fRange, fRange);
         m_pcKilobotCustomCommunicationEntity =
            new CKilobotCustomCommunicationEntity(this,
                                            "kilocomm_0",
                                            9,
                                            fRange,
                                            cCommAnchor,
                                            *m_pcEmbodiedEntity);
         AddComponent(*m_pcKilobotCustomCommunicationEntity);
         /* Ground sensor equipped entity */
         m_pcGroundSensorEquippedEntity =
            new CGroundSensorEquippedEntity(this, "ground_0");
         AddComponent(*m_pcGroundSensorEquippedEntity);
         m_pcGroundSensorEquippedEntity->AddSensor(KILOBOT_CUSTOM_GROUND_SENSOR_0_OFFSET,
                                                   CGroundSensorEquippedEntity::TYPE_BLACK_WHITE,
                                                   m_pcEmbodiedEntity->GetOriginAnchor());
         m_pcGroundSensorEquippedEntity->AddSensor(KILOBOT_CUSTOM_GROUND_SENSOR_1_OFFSET,
                                                   CGroundSensorEquippedEntity::TYPE_GRAYSCALE,
                                                   m_pcEmbodiedEntity->GetOriginAnchor());
         /* Controllable entity. It must be the last one, for
            actuators/sensors to link to composing entities
            correctly */
         m_pcControllableEntity = new CControllableEntity(this);
         AddComponent(*m_pcControllableEntity);
         m_pcControllableEntity->Init(GetNode(t_tree, "controller"));
         /* Update components */
         UpdateComponents();
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Failed to initialize entity \"" << GetId() << "\".", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CKilobotCustomEntity::Reset() {
      /* Reset all components */
      CComposableEntity::Reset();
      /* Update components */
      UpdateComponents();
   }

   /****************************************/
   /****************************************/

   void CKilobotCustomEntity::Destroy() {
      CComposableEntity::Destroy();
   }

   /****************************************/
   /****************************************/

#define UPDATE(COMPONENT) if(COMPONENT->IsEnabled()) COMPONENT->Update();

   void CKilobotCustomEntity::UpdateComponents() {
      UPDATE(m_pcKilobotCustomCommunicationEntity);
      UPDATE(m_pcLEDEquippedEntity);
   }

   /****************************************/
   /****************************************/

   REGISTER_ENTITY(CKilobotCustomEntity,
                   "kilobot_custom",
                   "Carlo Pinciroli [ilpincy@gmail.com] - Vito Trianni [vito.trianni@istc.cnr.it",
                   "1.0",
                   "A customized Kilobot robot with ground sensor and compass",
                   "The Kilobot is a low-cost robot designed to make testing collective algorithms\n"
                   "on hundreds or thousands (\"kilos\") of robots accessible to robotics\n"
                   "researchers.\n"
                   "For more information, refer to the dedicated web page\n"
                   "(http://www.eecs.harvard.edu/ssr/projects/progSA/kilobot.html).\n\n"
                   "REQUIRED XML CONFIGURATION\n\n"
                   "  <arena ...>\n"
                   "    ...\n"
                   "    <kilobot_custom id=\"fb0\">\n"
                   "      <body position=\"0.4,2.3,0.25\" orientation=\"45,0,0\" />\n"
                   "      <controller config=\"mycntrl\" />\n"
                   "    </kilobot_custom>\n"
                   "    ...\n"
                   "  </arena>\n\n"
                   "The 'id' attribute is necessary and must be unique among the entities. If two\n"
                   "entities share the same id, initialization aborts.\n"
                   "The 'body/position' attribute specifies the position of the bottom point of the\n"
                   "Kilobot in the arena. When the robot is untranslated and unrotated, the\n"
                   "bottom point is in the origin and it is defined as the middle point between\n"
                   "the two actuated pins on the XY plane and the lowest point of the robot on the\n"
                   "Z axis, that is the point where the robot touches the floor. The attribute\n"
                   "values are in the X,Y,Z order.\n"
                   "The 'body/orientation' attribute specifies the orientation of the Kilobot. All\n"
                   "rotations are performed with respect to the bottom point. The order of the\n"
                   "angles is Z,Y,X, which means that the first number corresponds to the rotation\n"
                   "around the Z axis, the second around Y and the last around X. This reflects\n"
                   "the internal convention used in ARGoS, in which rotations are performed in\n"
                   "that order. Angles are expressed in degrees. When the robot is unrotated, it\n"
                   "is oriented along the X axis.\n"
                   "The 'controller/config' attribute is used to assign a controller to the\n"
                   "Kilobot. The value of the attribute must be set to the id of a previously\n"
                   "defined controller. Controllers are defined in the <controllers> XML subtree.\n\n"
                   "OPTIONAL XML CONFIGURATION\n\n"
                   "You can set the emission range of the communication system. By default, a\n"
                   "message sent by a Kilobot can be received up to 10cm. By using the\n"
                   "'communication_range' attribute, you can change it to, i.e., 15cm as follows:\n\n"
                   "  <arena ...>\n"
                   "    ...\n"
                   "    <kilobot_custom id=\"fb0\" communication_range=\"0.15\">\n"
                   "      <body position=\"0.4,2.3,0.25\" orientation=\"45,0,0\" />\n"
                   "      <controller config=\"mycntrl\" />\n"
                   "    </kilobot_custom>\n"
                   "    ...\n"
                   "  </arena>\n\n"
                   ,
                   "Under development"
      );

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_SPACE_OPERATIONS_ON_COMPOSABLE(CKilobotCustomEntity);

   /****************************************/
   /****************************************/

}
