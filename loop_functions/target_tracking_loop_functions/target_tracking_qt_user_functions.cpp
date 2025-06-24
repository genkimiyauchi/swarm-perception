#include "target_tracking_qt_user_functions.h"
#include <controllers/robot/robot.h>
#include <argos3/core/simulator/entity/controllable_entity.h>
#include <QPainter>

using namespace argos;

/****************************************/
/****************************************/

CTargetTrackingQTUserFunctions::CTargetTrackingQTUserFunctions() {
   m_pcTargetTrackingLoopFunctions = static_cast<CTargetTrackingLoopFunctions *>(
      &CSimulator::GetInstance().GetLoopFunctions());
   m_bDrawRobotLabel = m_pcTargetTrackingLoopFunctions->IsDrawRobotLabel();
   m_unNumRobots = m_pcTargetTrackingLoopFunctions->GetNumRobots();

   RegisterUserFunction<CTargetTrackingQTUserFunctions,CEPuckEntity>(&CTargetTrackingQTUserFunctions::Draw);
}

/****************************************/
/****************************************/

void CTargetTrackingQTUserFunctions::Draw(CEPuckEntity& c_entity) {
   CRobot& cController = dynamic_cast<CRobot&>(c_entity.GetControllableEntity().GetController());

   if(m_bDrawRobotLabel) {
      // std::string label = c_entity.GetId().c_str();

      std::string state = cController.GetState();
      std::string label;
      CColor color = CColor::BLACK;
      if(state == "RANDOM_WALK") {
         label = "Searching";
         color = CColor::BLUE;
      }
      else if(state == "BROADCAST_WALK") {
         label = "Sharing target";
         color = CColor::RED;
      }
      else if(state == "BROADCAST_HOMING") {
         label = "Moving to target";
         color = CColor::GREEN;
      }
      else if(state == "IN_TARGET") {
         label = "Moving to target";
         color = CColor::GREEN;
      }
      else {
         label = "Unknown";
      }

      DrawText(CVector3(0.0, 0.0, 0.2), label, color);

      /* Draw RAB range */
      if(state == "BROADCAST_WALK" || state == "BROADCAST_HOMING" || state == "IN_TARGET") {
         Real fRABRange = cController.GetRABRange();
         DrawCircle(CVector3(0.0, 0.0, 0.001), CQuaternion(), fRABRange, color, false);
      }
   }
}

/****************************************/
/****************************************/

void CTargetTrackingQTUserFunctions::DrawOverlay(QPainter& c_painter) {

   size_t unNumRobotsInTarget = m_pcTargetTrackingLoopFunctions->GetNumRobotsInTarget();

   /* Text position */
   int textX = 310;//410;
   int textX_offset = 460;//370;
   int textY = 50;//60;

   /* Draw the total number of robots */
   c_painter.setPen(Qt::black); // Set text color
   QFont font = c_painter.font();
   font.setPointSize(20); // Set font size
   font.setBold(true); // Default font style
   c_painter.setFont(font);
   c_painter.drawText(textX, textY, QString("Number of robots in target:"));

   // c_painter.setPen(Qt::black); // Set text color
   // font.setBold(true); // Bold font for %1/%2
   // c_painter.setFont(font);
   c_painter.drawText(textX + textX_offset, textY, QString("%1 / %2")
                      .arg(unNumRobotsInTarget)
                      .arg(m_unNumRobots));
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CTargetTrackingQTUserFunctions, "target_tracking_qt_user_functions")
