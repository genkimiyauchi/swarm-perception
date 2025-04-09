#ifndef TARGET_TRACKING_QT_USER_FUNCTIONS_H
#define TARGET_TRACKING_QT_USER_FUNCTIONS_H

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <loop_functions/target_tracking_loop_functions/target_tracking_loop_functions.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>

using namespace argos;

class CTargetTrackingQTUserFunctions : public CQTOpenGLUserFunctions {

public:

   CTargetTrackingQTUserFunctions();

   virtual ~CTargetTrackingQTUserFunctions() {}

   void Draw(CEPuckEntity& c_entity);

private:

   CTargetTrackingLoopFunctions *m_pcTargetTrackingLoopFunctions;
   
   /**
    * True when we want to print the hop count of each robot
    */
   bool m_bDrawRobotLabel;

};

#endif
