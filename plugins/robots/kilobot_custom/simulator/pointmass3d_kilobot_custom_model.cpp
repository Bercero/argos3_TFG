/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_differentialsteering_control.cpp>
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 */

#include "pointmass3d_kilobot_custom_model.h"
#include "kilobot_custom_measures.h"
#include <argos3/core/utility/math/cylinder.h>

namespace argos {

   enum KILOBOT_CUSTOM_WHEELS {
      KILOBOT_CUSTOM_LEFT_WHEEL = 0,
      KILOBOT_CUSTOM_RIGHT_WHEEL = 1
   };

   /****************************************/
   /****************************************/

   CPointMass3DKilobotCustomModel::CPointMass3DKilobotCustomModel(CPointMass3DEngine& c_engine,
                                                      CKilobotCustomEntity& c_kilobot_custom) :
      CPointMass3DModel(c_engine, c_kilobot_custom.GetEmbodiedEntity()),
      m_cWheeledEntity(c_kilobot_custom.GetWheeledEntity()),
      m_fCurrentWheelVelocity(m_cWheeledEntity.GetWheelVelocities()) {
      /* Register the origin anchor update method */
      RegisterAnchorMethod(GetEmbodiedEntity().GetOriginAnchor(),
                           &CPointMass3DKilobotCustomModel::UpdateOriginAnchor);
      /* Get initial rotation */
      CRadians cTmp1, cTmp2;
      GetEmbodiedEntity().GetOriginAnchor().Orientation.ToEulerAngles(m_cYaw, cTmp1, cTmp2);
   }

   /****************************************/
   /****************************************/

   void CPointMass3DKilobotCustomModel::Reset() {
      CPointMass3DModel::Reset();
      CRadians cTmp1, cTmp2;
      GetEmbodiedEntity().GetOriginAnchor().Orientation.ToEulerAngles(m_cYaw, cTmp1, cTmp2);
      m_fAngularVelocity = 0.0;
   }

   /****************************************/
   /****************************************/


   void CPointMass3DKilobotCustomModel::UpdateFromEntityStatus() {
      m_cVelocity.Set((m_fCurrentWheelVelocity[KILOBOT_CUSTOM_RIGHT_WHEEL] + m_fCurrentWheelVelocity[KILOBOT_CUSTOM_LEFT_WHEEL])*0.5, 0.0, 0.0);
      m_cVelocity.RotateZ(m_cYaw);
      m_fAngularVelocity = (m_fCurrentWheelVelocity[KILOBOT_CUSTOM_RIGHT_WHEEL] - m_fCurrentWheelVelocity[KILOBOT_CUSTOM_LEFT_WHEEL]) / KILOBOT_CUSTOM_INTERPIN_DISTANCE;
   }

   /****************************************/
   /****************************************/

   void CPointMass3DKilobotCustomModel::Step() {
      m_cPosition += m_cVelocity * m_cPM3DEngine.GetPhysicsClockTick();
      m_cYaw += CRadians(m_fAngularVelocity * m_cPM3DEngine.GetPhysicsClockTick());
   }

   /****************************************/
   /****************************************/

   void CPointMass3DKilobotCustomModel::CalculateBoundingBox() {
      GetBoundingBox().MinCorner.Set(
         GetEmbodiedEntity().GetOriginAnchor().Position.GetX() - KILOBOT_CUSTOM_RADIUS,
         GetEmbodiedEntity().GetOriginAnchor().Position.GetY() - KILOBOT_CUSTOM_RADIUS,
         GetEmbodiedEntity().GetOriginAnchor().Position.GetZ());
      GetBoundingBox().MaxCorner.Set(
         GetEmbodiedEntity().GetOriginAnchor().Position.GetX() + KILOBOT_CUSTOM_RADIUS,
         GetEmbodiedEntity().GetOriginAnchor().Position.GetY() + KILOBOT_CUSTOM_RADIUS,
         GetEmbodiedEntity().GetOriginAnchor().Position.GetZ() + KILOBOT_CUSTOM_HEIGHT);
   }

   /****************************************/
   /****************************************/

   bool CPointMass3DKilobotCustomModel::CheckIntersectionWithRay(Real& f_t_on_ray,
                                                           const CRay3& c_ray) const {
      CCylinder m_cShape(KILOBOT_CUSTOM_RADIUS,
                         KILOBOT_CUSTOM_HEIGHT,
                         m_cPosition,
                         CVector3::Z);
      bool bIntersects = m_cShape.Intersects(f_t_on_ray, c_ray);
      return bIntersects;
   }

   /****************************************/
   /****************************************/

   void CPointMass3DKilobotCustomModel::UpdateOriginAnchor(SAnchor& s_anchor) {
      s_anchor.Position = m_cPosition;
      s_anchor.Orientation = CQuaternion(m_cYaw, CVector3::Z);
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_POINTMASS3D_OPERATIONS_ON_ENTITY(CKilobotCustomEntity, CPointMass3DKilobotCustomModel);

   /****************************************/
   /****************************************/

}
