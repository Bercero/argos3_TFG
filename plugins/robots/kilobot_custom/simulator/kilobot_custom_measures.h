/**
 * @file <argos3/plugins/robots/kilobot_custom/simulator/kilobot_custom_measures.h>
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 * @author Vito Trianni - <vito.trianni@istc.cnr.it>
 */

#ifndef KILOBOT_CUSTOM_MEASURES_H
#define KILOBOT_CUSTOM_MEASURES_H

#include <argos3/core/utility/datatypes/datatypes.h>
#include <argos3/core/utility/math/angles.h>
#include <argos3/core/utility/math/vector3.h>

namespace argos {

   /* KilobotCustom measures */
   static const Real KILOBOT_CUSTOM_BODY_HEIGHT            = 0.0127;
   static const Real KILOBOT_CUSTOM_PIN_HEIGHT             = 0.0217;
   static const Real KILOBOT_CUSTOM_PIN_RADIUS             = 0.0007;
   static const Real KILOBOT_CUSTOM_INTERPIN_DISTANCE      = 0.025;
   static const Real KILOBOT_CUSTOM_HALF_INTERPIN_DISTANCE = KILOBOT_CUSTOM_INTERPIN_DISTANCE * 0.5;
   static const Real KILOBOT_CUSTOM_FRONT_PIN_DISTANCE     = 0.025;
   static const Real KILOBOT_CUSTOM_RADIUS                 = 0.0165;
   static const Real KILOBOT_CUSTOM_ECCENTRICITY           = 0.0092;
   static const Real KILOBOT_CUSTOM_HEIGHT                 = KILOBOT_CUSTOM_PIN_HEIGHT + KILOBOT_CUSTOM_BODY_HEIGHT;
   static const Real KILOBOT_CUSTOM_MASS                   = 0.01;

   /* RGB LED */
   static const Real KILOBOT_CUSTOM_PIN_WHEEL_RADIUS       = 0.001;
   static const CRadians KILOBOT_CUSTOM_LED_ANGLE          = CRadians(ARGOS_PI / 6.0);
   static const Real KILOBOT_CUSTOM_LED_ELEVATION          = KILOBOT_CUSTOM_HEIGHT;
   static const Real KILOBOT_CUSTOM_LED_RADIUS             = KILOBOT_CUSTOM_RADIUS * 0.4;
   static const Real KILOBOT_CUSTOM_LED_HEIGHT             = KILOBOT_CUSTOM_RADIUS * 0.05;

   /* Light Sensor */
   static const CRadians KILOBOT_CUSTOM_LIGHT_SENSOR_ANGLE  = ToRadians(CDegrees(100.0));
   static const Real KILOBOT_CUSTOM_LIGHT_SENSOR_ELEVATION  = KILOBOT_CUSTOM_HEIGHT;
   static const Real KILOBOT_CUSTOM_LIGHT_SENSOR_RADIUS     = KILOBOT_CUSTOM_RADIUS - 0.002;
   static const Real KILOBOT_CUSTOM_LIGHT_SENSOR_RANGE      = 0.1;
   static const CVector3 KILOBOT_CUSTOM_LIGHT_SENSOR_OFFSET =
      CVector3(0.0,
               0.0,
               KILOBOT_CUSTOM_LIGHT_SENSOR_ELEVATION) +
      CVector3(KILOBOT_CUSTOM_LIGHT_SENSOR_RADIUS * Cos(KILOBOT_CUSTOM_LIGHT_SENSOR_ANGLE),
               KILOBOT_CUSTOM_LIGHT_SENSOR_RADIUS * Sin(KILOBOT_CUSTOM_LIGHT_SENSOR_ANGLE),
               0.0f) +
      CVector3(KILOBOT_CUSTOM_ECCENTRICITY,
               0.0,
               0.0);

   /* Ground sensors */
   static const CVector2 KILOBOT_CUSTOM_GROUND_SENSOR_0_OFFSET =
      CVector2(0.0,
               0.0);
   static const CVector2 KILOBOT_CUSTOM_GROUND_SENSOR_1_OFFSET =
      CVector2(KILOBOT_CUSTOM_RADIUS,
               0.0);
   /* Communication RAB */
   static const Real KILOBOT_CUSTOM_RAB_ELEVATION          = KILOBOT_CUSTOM_HEIGHT + 0.001;



}


#endif
