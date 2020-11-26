/**
 * @file <argos3/plugins/robots/kilobot_custom/simulator/dynamics2d_kilobot_custom_model.cpp>
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 * @author Vito Trianni - <vito.trianni@istc.cnr.it>
 */

#include "dynamics2d_kilobot_custom_model.h"
#include "kilobot_custom_measures.h"
#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_engine.h>

namespace argos {

   /****************************************/
   /****************************************/

   static const Real KILOBOT_CUSTOM_MAX_FORCE  = 0.001f;
   static const Real KILOBOT_CUSTOM_MAX_TORQUE = 0.001f;
   static const Real KILOBOT_CUSTOM_FRICTION   = 2.5f;

   enum KILOBOT_CUSTOM_WHEELS {
      KILOBOT_CUSTOM_LEFT_WHEEL = 0,
      KILOBOT_CUSTOM_RIGHT_WHEEL = 1
   };

   /****************************************/
   /****************************************/

   CDynamics2DKilobotCustomModel::CDynamics2DKilobotCustomModel(CDynamics2DEngine& c_engine,
                                                    CKilobotCustomEntity& c_entity) :
      CDynamics2DSingleBodyObjectModel(c_engine, c_entity),
      m_cKilobotCustomEntity(c_entity),
      m_cWheeledEntity(m_cKilobotCustomEntity.GetWheeledEntity()),
      m_cDiffSteering(c_engine,
                      KILOBOT_CUSTOM_MAX_FORCE,
                      KILOBOT_CUSTOM_MAX_TORQUE,
                      KILOBOT_CUSTOM_INTERPIN_DISTANCE,
                      c_entity.GetConfigurationNode()),
      m_fCurrentWheelVelocity(m_cWheeledEntity.GetWheelVelocities()) {
      /* Parse the XML file to check if friction was specified */
      cpFloat fFriction = KILOBOT_CUSTOM_FRICTION;
      if(c_entity.GetConfigurationNode() &&
         NodeExists(*c_entity.GetConfigurationNode(), "dynamics2d")) {
         TConfigurationNode& tDyn2D = GetNode(*c_entity.GetConfigurationNode(), "dynamics2d");
         GetNodeAttributeOrDefault(tDyn2D, "friction", fFriction, fFriction);
      }
      /* Create the actual body with initial position and orientation */
      cpBody* ptBody =
         cpSpaceAddBody(GetDynamics2DEngine().GetPhysicsSpace(),
                        cpBodyNew(KILOBOT_CUSTOM_MASS,
                                  cpMomentForCircle(KILOBOT_CUSTOM_MASS,
                                                    0.0f,
                                                    KILOBOT_CUSTOM_RADIUS + KILOBOT_CUSTOM_RADIUS,
                                                    cpv(KILOBOT_CUSTOM_ECCENTRICITY,0))));
      const CVector3& cPosition = GetEmbodiedEntity().GetOriginAnchor().Position;
      ptBody->p = cpv(cPosition.GetX(), cPosition.GetY());
      CRadians cXAngle, cYAngle, cZAngle;
      GetEmbodiedEntity().GetOriginAnchor().Orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);
      cpBodySetAngle(ptBody, cZAngle.GetValue());
      /* Create the actual body shape */
      cpShape* ptShape =
         cpSpaceAddShape(GetDynamics2DEngine().GetPhysicsSpace(),
                         cpCircleShapeNew(ptBody,
                                          KILOBOT_CUSTOM_RADIUS,
                                          cpv(KILOBOT_CUSTOM_ECCENTRICITY,0)));
      ptShape->e = 0.0;       // No elasticity
      ptShape->u = fFriction; // Friction
      /* Constrain the body to follow the diff steering control */
      m_cDiffSteering.AttachTo(ptBody);
      /* Set the body so that the default methods work as expected */
      SetBody(ptBody, KILOBOT_CUSTOM_HEIGHT);
      /* Set the anchor updaters */
      RegisterAnchorMethod<CDynamics2DKilobotCustomModel>(
         GetEmbodiedEntity().GetAnchor("light"),
         &CDynamics2DKilobotCustomModel::UpdateLightAnchor);
      RegisterAnchorMethod<CDynamics2DKilobotCustomModel>(
         GetEmbodiedEntity().GetAnchor("comm"),
         &CDynamics2DKilobotCustomModel::UpdateCommAnchor);
   }

   /****************************************/
   /****************************************/

   CDynamics2DKilobotCustomModel::~CDynamics2DKilobotCustomModel() {
      m_cDiffSteering.Detach();
   }

  
   /****************************************/
   /****************************************/

   void CDynamics2DKilobotCustomModel::Reset() {
      CDynamics2DSingleBodyObjectModel::Reset();
      m_cDiffSteering.Reset();
   }

   /****************************************/
   /****************************************/

   void CDynamics2DKilobotCustomModel::UpdateFromEntityStatus() {
      /* Do we want to move? */
      if((m_fCurrentWheelVelocity[KILOBOT_CUSTOM_LEFT_WHEEL] != 0.0f) ||
         (m_fCurrentWheelVelocity[KILOBOT_CUSTOM_RIGHT_WHEEL] != 0.0f)) {
         m_cDiffSteering.SetWheelVelocity(m_fCurrentWheelVelocity[KILOBOT_CUSTOM_LEFT_WHEEL],
                                          m_fCurrentWheelVelocity[KILOBOT_CUSTOM_RIGHT_WHEEL]);
      }
      else {
         /* No, we don't want to move - zero all speeds */
         m_cDiffSteering.Reset();
      }
   }

   /****************************************/
   /****************************************/

   void CDynamics2DKilobotCustomModel::UpdateLightAnchor(SAnchor& s_anchor) {
      /* Start in origin, put anchor in offset */
      s_anchor.Position = s_anchor.OffsetPosition;
      /* Rotate anchor by body orientation in world */
      s_anchor.Orientation.FromAngleAxis(CRadians(GetBody()->a), CVector3::Z);
      s_anchor.Position.Rotate(s_anchor.Orientation);
      /* Translate anchor by body position in world */
      s_anchor.Position.SetX(s_anchor.Position.GetX() + GetBody()->p.x);
      s_anchor.Position.SetY(s_anchor.Position.GetY() + GetBody()->p.y);
   }

   /****************************************/
   /****************************************/

   void CDynamics2DKilobotCustomModel::UpdateCommAnchor(SAnchor& s_anchor) {
      /* Start in origin, put anchor in offset */
      s_anchor.Position = s_anchor.OffsetPosition;
      /* Rotate anchor by body orientation in world */
      s_anchor.Orientation.FromAngleAxis(CRadians(GetBody()->a), CVector3::Z);
      s_anchor.Position.Rotate(s_anchor.Orientation);
      /* Translate anchor by body position in world */
      s_anchor.Position.SetX(s_anchor.Position.GetX() + GetBody()->p.x);
      s_anchor.Position.SetY(s_anchor.Position.GetY() + GetBody()->p.y);
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_DYNAMICS2D_OPERATIONS_ON_ENTITY(CKilobotCustomEntity, CDynamics2DKilobotCustomModel);

   /****************************************/
   /****************************************/

}
