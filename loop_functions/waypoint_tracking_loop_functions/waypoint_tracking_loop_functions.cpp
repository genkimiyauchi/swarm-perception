#include "waypoint_tracking_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_render.h>
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_widget.h>
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
        
        /*
        * Parse the configuration file
        */
        config = t_node;
        TConfigurationNode& tSettings = GetNode(config, "output");

        /* Set the frame grabbing settings */
        GetNodeAttributeOrDefault(tSettings, "frame_grabbing", m_bFrameGrabbing, false);
        GetNodeAttributeOrDefault(tSettings, "camera_index", m_unCameraIndex, (UInt32)0);

        /* Target position */
        TConfigurationNode& etTargets = GetNode(config, "targets");
        TConfigurationNodeIterator itTargets;

        for(itTargets = itTargets.begin(&etTargets);
            itTargets != itTargets.end();
            ++itTargets) {

            /* Get current node (team/custom_team) */
            TConfigurationNode& tTarget = *itTargets;

            LOG << "HI " << itTargets->Value() << std::endl;

            /* Get target positions */
            if(itTargets->Value() == "target") {
                LOG << "IN " << itTargets->Value() << std::endl;

                CVector2 cCenter;
                Real fRadius;
                GetNodeAttributeOrDefault(tTarget, "center", cCenter, CVector2());
                GetNodeAttributeOrDefault(tTarget, "radius", fRadius, 0.0);
                /* Add target to vector */
                m_vecTargets.push_back({cCenter, fRadius});
            }
        }

        /* print all vecTargets */
        for(auto& target : m_vecTargets) {
            LOG << "Target: " << target.first << " Radius: " << target.second << std::endl;
        }
    }
    catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error parsing loop functions!", ex);
    }

    /* Get a pointer to the floor entity */
    m_pcFloor = &GetSpace().GetFloorEntity();

    /* Create a new RNG */
    m_pcRNG = CRandom::CreateRNG("argos");

    // m_cWaypoint.Set(1.0f, 1.0f); // TEMP hard-coded value
    // m_fWaypointRadius = 0.1f; // TEMP hard-coded value
    // LOG << "Waypoint: " << m_cWaypoint << std::endl;

    /* Update floor */
    m_pcFloor->SetChanged();
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
    /* If point is within the area of targets, return red */
    for(const auto& target : m_vecTargets) {
        if((c_position_on_plane - target.first).SquareLength() < target.second) {
            return CColor(255,191,191);
        }
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

void CWaypointTrackingLoopFunctions::PostStep() {

    /* Grab frame */
    if(m_bFrameGrabbing) {
        CQTOpenGLRender& render = dynamic_cast<CQTOpenGLRender&>(GetSimulator().GetVisualization());
        CQTOpenGLWidget& widget = render.GetMainWindow().GetOpenGLWidget();
        widget.SetCamera(m_unCameraIndex);
        widget.SetGrabFrame(m_bFrameGrabbing);
    }
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CWaypointTrackingLoopFunctions, "waypoint_tracking_loop_functions")
