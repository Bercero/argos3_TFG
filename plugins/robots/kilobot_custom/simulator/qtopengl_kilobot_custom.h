/**
 * @file <argos3/plugins/robots/kilobot_custom/simulator/qtopengl_kilobot_custom.h>
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 * @author Vito Trianni - <vito.trianni@istc.cnr.it>
 */

#ifndef QTOPENGL_KILOBOT_CUSTOM_H
#define QTOPENGL_KILOBOT_CUSTOM_H

namespace argos {
   class CQTOpenGLKilobotCustom;
   class CKilobotCustomEntity;
}

#ifdef __APPLE__
#include <gl.h>
#else
#include <GL/gl.h>
#endif

namespace argos {

   class CQTOpenGLKilobotCustom {

   public:

      CQTOpenGLKilobotCustom();

      virtual ~CQTOpenGLKilobotCustom();

      virtual void Draw(CKilobotCustomEntity& c_entity);

   protected:

      /** Renders a materialless wheel
          - centered in 0,0,0
          - rotation axis: Y
       */
      void MakeWheel();

      /** Sets a white plastic material */
      void SetWhitePlasticMaterial();
      /** Sets a black tire material */
      void SetBlackTireMaterial();
      /** Sets a circuit board material */
      void SetCircuitBoardMaterial();
      /** Sets a colored LED material */
      void SetLEDMaterial(GLfloat f_red, GLfloat f_green, GLfloat f_blue);

      /** Renders the wheels */
      void RenderWheel();
      /** Renders the base (apart from the wheels) */
      void RenderBase();
      /** Renders the LED */
      void RenderLED();

   private:

      /** Start of the display list index */
      GLuint m_unLists;

      /** List corresponding to the materialless wheel */
      GLuint m_unBasicWheelList;

      /** kilobot_custom wheel */
      GLuint m_unWheelList;
      /** kilobot_custom base module */
      GLuint m_unBaseList;
      /** kilobot_custom LED */
      GLuint m_unLEDList;

      /** Number of vertices to display the round parts
          (chassis, etc.) */
      GLuint m_unVertices;

   };

}

#endif
