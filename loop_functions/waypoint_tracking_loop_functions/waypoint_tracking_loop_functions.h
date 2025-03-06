#ifndef WAYPOINT_TRACKING_LOOP_FUNCTIONS_H
#define WAYPOINT_TRACKING_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/rng.h>

using namespace argos;

class CWaypointTrackingLoopFunctions : public CLoopFunctions {

public:

   CWaypointTrackingLoopFunctions();
   virtual ~CWaypointTrackingLoopFunctions() {}

   virtual void Init(TConfigurationNode& t_tree);
   virtual void Reset();
   virtual void Destroy();
   virtual CColor GetFloorColor(const CVector2& c_position_on_plane);
   virtual void PreStep();

private:

   CFloorEntity* m_pcFloor;
   CRandom::CRNG* m_pcRNG;

   CVector2 m_cWaypoint;
   Real m_fWaypointRadius;

   std::string m_strOutput;
   std::ofstream m_cOutput;

};

#endif
