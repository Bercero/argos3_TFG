/**
 * @file <argos3/plugins/simulator/robots/kilo-bot/simulator/pointmass3d/pointmass3d_kilobot_custom_model.h>
 *
 * @author Carlo Pinciroli - <cpinciro@ulb.ac.be>
 */

#ifndef POINTMASS3D_KILOBOT_CUSTOM_H
#define POINTMASS3D_KILOBOT_CUSTOM_H

namespace argos {
   class CPointMass3DEngine;
   class CPointMass3DKilobotCustomModel;
   class CKilobotCustomEntity;
}

#include <argos3/plugins/simulator/physics_engines/pointmass3d/pointmass3d_model.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/plugins/robots/kilobot_custom/simulator/kilobot_custom_entity.h>

namespace argos {

   class CPointMass3DKilobotCustomModel : public CPointMass3DModel {

   public:

      CPointMass3DKilobotCustomModel(CPointMass3DEngine& c_engine,
                               CKilobotCustomEntity& c_kilobot_custom);

      virtual ~CPointMass3DKilobotCustomModel() {}

      virtual void Reset();

      virtual void UpdateFromEntityStatus();
      virtual void Step();

      virtual void CalculateBoundingBox();

      virtual bool CheckIntersectionWithRay(Real& f_t_on_ray,
                                            const CRay3& c_ray) const;

      virtual void UpdateOriginAnchor(SAnchor& s_anchor);

   private:

      /** Reference to the wheeled entity */
      CWheeledEntity& m_cWheeledEntity;

      /** The yaw of the kilo-bot entity */
      CRadians m_cYaw;

      /** The yaw of the kilo-bot entity */
      Real m_fAngularVelocity;

      /** Current wheel velocity */
      const Real* m_fCurrentWheelVelocity;
   };

}

#endif
