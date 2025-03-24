#ifndef WAYPOINT_TRACKING_QT_USER_FUNCTIONS_H
#define WAYPOINT_TRACKING_QT_USER_FUNCTIONS_H

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>

using namespace argos;

class CWaypointTrackingQTUserFunctions : public CQTOpenGLUserFunctions {

public:

   CWaypointTrackingQTUserFunctions();

   virtual ~CWaypointTrackingQTUserFunctions() {}

   void Draw(CEPuckEntity& c_entity);
   
};

#endif
