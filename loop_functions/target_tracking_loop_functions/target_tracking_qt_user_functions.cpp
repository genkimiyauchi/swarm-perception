#include "target_tracking_qt_user_functions.h"
#include <controllers/robot/robot.h>
#include <argos3/core/simulator/entity/controllable_entity.h>

using namespace argos;

/****************************************/
/****************************************/

CTargetTrackingQTUserFunctions::CTargetTrackingQTUserFunctions() {
   RegisterUserFunction<CTargetTrackingQTUserFunctions,CEPuckEntity>(&CTargetTrackingQTUserFunctions::Draw);
}

/****************************************/
/****************************************/

void CTargetTrackingQTUserFunctions::Draw(CEPuckEntity& c_entity) {
   CRobot& cController = dynamic_cast<CRobot&>(c_entity.GetControllableEntity().GetController());

    std::string id = c_entity.GetId().c_str();

    DrawText(CVector3(0.0, 0.0, 0.2), id);

}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CTargetTrackingQTUserFunctions, "target_tracking_qt_user_functions")
