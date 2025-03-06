#include "waypoint_tracking_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>
#include <controllers/epuck_waypoint_tracking/epuck_waypoint_tracking.h>

/****************************************/
/****************************************/

CWaypointTrackingLoopFunctions::CWaypointTrackingLoopFunctions() :
   m_pcFloor(NULL),
   m_pcRNG(NULL) {
}

/****************************************/
/****************************************/

void CWaypointTrackingLoopFunctions::Init(TConfigurationNode& t_node) {
   try {
      // TConfigurationNode& tForaging = GetNode(t_node, "foraging");
      /* Get a pointer to the floor entity */
      m_pcFloor = &GetSpace().GetFloorEntity();
      // /* Get the number of food items we want to be scattered from XML */
      // UInt32 unFoodItems;
      // GetNodeAttribute(tForaging, "items", unFoodItems);
      // /* Get the number of food items we want to be scattered from XML */
      // GetNodeAttribute(tForaging, "radius", m_fFoodSquareRadius);
      // m_fFoodSquareRadius *= m_fFoodSquareRadius;
      /* Create a new RNG */
      m_pcRNG = CRandom::CreateRNG("argos");
      // /* Distribute uniformly the items in the environment */
      // for(UInt32 i = 0; i < unFoodItems; ++i) {
      //    m_cFoodPos.push_back(
      //       CVector2(m_pcRNG->Uniform(m_cForagingArenaSideX),
      //                m_pcRNG->Uniform(m_cForagingArenaSideY)));
      // }
      // /* Get the output file name from XML */
      // GetNodeAttribute(tForaging, "output", m_strOutput);
      // /* Open the file, erasing its contents */
      // m_cOutput.open(m_strOutput.c_str(), std::ios_base::trunc | std::ios_base::out);
      // m_cOutput << "# clock\twalking\tresting\tcollected_food\tenergy" << std::endl;
      // /* Get energy gain per item collected */
      // GetNodeAttribute(tForaging, "energy_per_item", m_unEnergyPerFoodItem);
      // /* Get energy loss per walking robot */
      // GetNodeAttribute(tForaging, "energy_per_walking_robot", m_unEnergyPerWalkingRobot);

      m_cWaypoint.Set(1.0f, 1.0f); // TEMP hard-coded value
      m_fWaypointRadius = 0.1f; // TEMP hard-coded value
      LOG << "Waypoint: " << m_cWaypoint << std::endl;

      m_pcFloor->SetChanged();
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error parsing loop functions!", ex);
   }
}

/****************************************/
/****************************************/

void CWaypointTrackingLoopFunctions::Reset() {
   // /* Zero the counters */
   // m_unCollectedFood = 0;
   // m_nEnergy = 0;
   // /* Close the file */
   // m_cOutput.close();
   // /* Open the file, erasing its contents */
   // m_cOutput.open(m_strOutput.c_str(), std::ios_base::trunc | std::ios_base::out);
   // m_cOutput << "# clock\twalking\tresting\tcollected_food\tenergy" << std::endl;
   // /* Distribute uniformly the items in the environment */
   // for(UInt32 i = 0; i < m_cFoodPos.size(); ++i) {
   //    m_cFoodPos[i].Set(m_pcRNG->Uniform(m_cForagingArenaSideX),
   //                      m_pcRNG->Uniform(m_cForagingArenaSideY));
   // }
}

/****************************************/
/****************************************/

void CWaypointTrackingLoopFunctions::Destroy() {
   /* Close the file */
   // m_cOutput.close();
}

/****************************************/
/****************************************/

CColor CWaypointTrackingLoopFunctions::GetFloorColor(const CVector2& c_position_on_plane) {
   /* If point is within waypoint radius, return green */
   if((c_position_on_plane - m_cWaypoint).SquareLength() < m_fWaypointRadius) {
      return CColor(255,191,191);
   }
   return CColor::WHITE;
}

/****************************************/
/****************************************/

void CWaypointTrackingLoopFunctions::PreStep() {

   LOG << "TIME: " << GetSpace().GetSimulationClock() << std::endl;

}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CWaypointTrackingLoopFunctions, "waypoint_tracking_loop_functions")
