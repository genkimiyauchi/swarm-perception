#include "target_tracking_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_render.h>
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_widget.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>
#include <controllers/robot/robot.h>

/****************************************/
/****************************************/

CTargetTrackingLoopFunctions::CTargetTrackingLoopFunctions() :
    m_pcFloor(NULL),
    m_pcRNG(NULL) {
}

/****************************************/
/****************************************/

void CTargetTrackingLoopFunctions::Init(TConfigurationNode& t_node) {
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

            /* Get target positions */
            if(itTargets->Value() == "target") {
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
            LOG << "[INFO] Target: " << target.first << " Radius: " << target.second << std::endl;
        }
    }
    catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error parsing loop functions!", ex);
    }

    /* Get a pointer to the floor entity */
    m_pcFloor = &GetSpace().GetFloorEntity();

    /* Create a new RNG */
    m_pcRNG = CRandom::CreateRNG("argos");

    /* Update floor */
    m_pcFloor->SetChanged();

    /* Access all e-puck entities in simulation and print their ids */
    CSpace::TMapPerType& m_cEPucks = GetSpace().GetEntitiesByType("e-puck");
    for(CSpace::TMapPerType::iterator itEpuck = m_cEPucks.begin();
        itEpuck != m_cEPucks.end();
        ++itEpuck) {

        CEPuckEntity& cEPuck = *any_cast<CEPuckEntity*>(itEpuck->second);
        CRobot& cController = dynamic_cast<CRobot&>(cEPuck.GetControllableEntity().GetController());
        UInt8 teamID = cController.GetTeamID();
        cController.SetTarget(m_vecTargets[teamID-1].first, m_vecTargets[teamID-1].second); // TEMP: Use team assigned target
    }
}

/****************************************/
/****************************************/

void CTargetTrackingLoopFunctions::Reset() {
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

void CTargetTrackingLoopFunctions::Destroy() {
    /* Close the file */
    // m_cOutput.close();
}

/****************************************/
/****************************************/

CColor CTargetTrackingLoopFunctions::GetFloorColor(const CVector2& c_position_on_plane) {
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

void CTargetTrackingLoopFunctions::PreStep() {

    LOG << "TIME: " << GetSpace().GetSimulationClock() << std::endl;

}

/****************************************/
/****************************************/

void CTargetTrackingLoopFunctions::PostStep() {

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

REGISTER_LOOP_FUNCTIONS(CTargetTrackingLoopFunctions, "target_tracking_loop_functions")
