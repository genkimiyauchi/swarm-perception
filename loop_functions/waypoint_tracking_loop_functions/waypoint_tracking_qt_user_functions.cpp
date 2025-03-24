#include "waypoint_tracking_qt_user_functions.h"
#include <controllers/epuck_waypoint_tracking/epuck_waypoint_tracking.h>
#include <argos3/core/simulator/entity/controllable_entity.h>

using namespace argos;

/****************************************/
/****************************************/

CWaypointTrackingQTUserFunctions::CWaypointTrackingQTUserFunctions() {
   RegisterUserFunction<CWaypointTrackingQTUserFunctions,CEPuckEntity>(&CWaypointTrackingQTUserFunctions::Draw);
}

/****************************************/
/****************************************/

void CWaypointTrackingQTUserFunctions::Draw(CEPuckEntity& c_entity) {
   CEPuckWaypointTracking& cController = dynamic_cast<CEPuckWaypointTracking&>(c_entity.GetControllableEntity().GetController());

    std::string id = c_entity.GetId().c_str();

    DrawText(CVector3(0.0, 0.0, 0.2), id);

}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CWaypointTrackingQTUserFunctions, "waypoint_tracking_qt_user_functions")
