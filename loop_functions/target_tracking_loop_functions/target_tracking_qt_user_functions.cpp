#include "target_tracking_qt_user_functions.h"
#include <controllers/robot/robot.h>
#include <argos3/core/simulator/entity/controllable_entity.h>

using namespace argos;

/****************************************/
/****************************************/

CTargetTrackingQTUserFunctions::CTargetTrackingQTUserFunctions() {
   m_pcTargetTrackingLoopFunctions = static_cast<CTargetTrackingLoopFunctions *>(
      &CSimulator::GetInstance().GetLoopFunctions());
   m_bDrawRobotLabel = m_pcTargetTrackingLoopFunctions->IsDrawRobotLabel();

   RegisterUserFunction<CTargetTrackingQTUserFunctions,CEPuckEntity>(&CTargetTrackingQTUserFunctions::Draw);
}

/****************************************/
/****************************************/

void CTargetTrackingQTUserFunctions::Draw(CEPuckEntity& c_entity) {
   CRobot& cController = dynamic_cast<CRobot&>(c_entity.GetControllableEntity().GetController());

   if(m_bDrawRobotLabel) {
      std::string id = c_entity.GetId().c_str();
      DrawText(CVector3(0.0, 0.0, 0.2), id);
   }
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CTargetTrackingQTUserFunctions, "target_tracking_qt_user_functions")
