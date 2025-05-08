#include "target_tracking_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_render.h>
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_widget.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>
#include <controllers/robot/robot.h>

/****************************************/
/****************************************/

/* Constants */
static const Real ARENA_SIZE_X = 2;
static const Real ARENA_SIZE_Y = 2;
static const Real WALL_WIDTH = 0.05;

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
        
        /* Get a pointer to the floor entity */
        m_pcFloor = &GetSpace().GetFloorEntity();

        /* Create a new RNG */
        m_pcRNG = CRandom::CreateRNG("argos");

        /*
        * Parse the configuration file
        */
        config = t_node;
        TConfigurationNode& tSettings = GetNode(config, "output");

        /* Set the frame grabbing settings */
        GetNodeAttributeOrDefault(tSettings, "frame_grabbing", m_bFrameGrabbing, false);
        GetNodeAttributeOrDefault(tSettings, "camera_index", m_unCameraIndex, (UInt32)0);

        TConfigurationNode& tDraw = GetNode(config, "draw");
        GetNodeAttributeOrDefault(tDraw, "robot_label", m_bDrawRobotLabel, true);

        /*** Target location ***/
        TConfigurationNode& tTarget = GetNode(config, "target");

        Real fRadius;
        CVector2 cCenter;

        GetNodeAttributeOrDefault(tTarget, "radius", fRadius, 0.25);

        /* Select a random target location within the arena */
        Real fX = m_pcRNG->Uniform(CRange<Real>(-ARENA_SIZE_X/2 + WALL_WIDTH/2 + fRadius, 
                                                 ARENA_SIZE_X/2 - WALL_WIDTH/2 - fRadius));
        Real fY = m_pcRNG->Uniform(CRange<Real>(-ARENA_SIZE_Y/2 + WALL_WIDTH/2 + fRadius,
                                                 ARENA_SIZE_Y/2 - WALL_WIDTH/2 - fRadius));
        
                                                 /* Round to 3 decimal places */
        fX = std::round(fX * 1000.0) / 1000.0;
        fY = std::round(fY * 1000.0) / 1000.0;
        cCenter.Set(fX, fY);

        m_vecTargets.push_back({cCenter, fRadius});

        /* print all vecTargets */
        for(auto& target : m_vecTargets) {
            LOG << "[INFO] Target: " << target.first << " Radius: " << target.second << std::endl;
        }

        /*** Teams ***/
        TConfigurationNode& etRobots = GetNode(config, "teams");
        /* Go through the nodes (teams) */
        TConfigurationNodeIterator itDistr;

        UInt8 teamID = 1;
        
        for(itDistr = itDistr.begin(&etRobots);
            itDistr != itDistr.end();
            ++itDistr) {
        
            /* Get current node (team/custom_team) */
            TConfigurationNode& tRobots = *itDistr;

            /* Get the controller */
            std::string strController;
            GetNodeAttribute(tRobots, "controller", strController);
            /* Get base name */
            std::string strBaseName;
            GetNodeAttributeOrDefault(tRobots, "base_name", strBaseName, std::string("ep"));
            /* Get base number */
            size_t unBaseNumber;
            GetNodeAttributeOrDefault(tRobots, "base_number", unBaseNumber, (size_t)1);
            /* Get the number of robots */
            UInt32 unRobots = 0;
            GetNodeAttribute(tRobots, "quantity", unRobots);
            /* Get the maximum number of trials for placing a robot */
            UInt32 unMaxTrials = 0;
            GetNodeAttributeOrDefault(tRobots, "max_trials", unMaxTrials, (UInt32)100);
            /* Get RAB range */
            Real fRABRange = 0.0f;
            GetNodeAttributeOrDefault(tRobots, "rab_range", fRABRange, (Real)0.8f);

            LOG << "[INFO] TEAM: " << teamID << std::endl;
            LOG << "[INFO] \t--Controller: " << strController << std::endl;
            LOG << "[INFO] \t--Base name: " << strBaseName << std::endl;
            LOG << "[INFO] \t--Number of robots: " << unRobots << std::endl;
            LOG << "[INFO] \t--Max trials: " << unMaxTrials << std::endl;
            LOG << "[INFO] \t--RAB range: " << fRABRange << std::endl;

            /* Place the robots */
            for(size_t i = 0; i < unRobots; ++i) {

                UInt32 unTrials = 0;
                CEPuckEntity* pcEP;
                std::ostringstream cEPId;
                CVector3 cEPPos;
                CQuaternion cEPRot;

                /* Make the id */
                cEPId.str("");
                cEPId << strBaseName << (i + unBaseNumber);
                /* Create the robot in the origin and add it to ARGoS space */
                pcEP = new CEPuckEntity(cEPId.str(),
                                        strController,
                                        CVector3(),
                                        CQuaternion(),
                                        fRABRange,
                                        MESSAGE_BYTE_SIZE,
                                        "");
                AddEntity(*pcEP);

                /* Set team ID */
                CRobot* cController = dynamic_cast<CRobot*>(&pcEP->GetControllableEntity().GetController());
                cController->SetTeamID(teamID);

                bool bDone = false;
                do {
                    /* Choose a random position and orientation */
                    ++unTrials;
                    cEPPos.Set(m_pcRNG->Uniform(CRange<Real>(-ARENA_SIZE_X/2 + WALL_WIDTH/2 + fRadius, 
                                                            ARENA_SIZE_X/2 - WALL_WIDTH/2 - fRadius)),
                            m_pcRNG->Uniform(CRange<Real>(-ARENA_SIZE_Y/2 + WALL_WIDTH/2 + fRadius, 
                                                            ARENA_SIZE_Y/2 - WALL_WIDTH/2 - fRadius)),
                            0.0f);
                    cEPRot.FromAngleAxis(m_pcRNG->Uniform(CRadians::UNSIGNED_RANGE), CVector3::Z);

                    /* Check the position is not in the target area */
                    CVector2 cEPPos2 = CVector2(cEPPos.GetX(), cEPPos.GetY());
                    if(Distance(cEPPos2, cCenter) < fRadius) {
                        LOG << "[INFO] Robot " << cEPId.str() << " is in the target area, retrying..." << std::endl;
                        continue;
                    }

                    bDone = MoveEntity(pcEP->GetEmbodiedEntity(), cEPPos, cEPRot);

                } while(!bDone && unTrials <= unMaxTrials);
                if(!bDone) {
                    THROW_ARGOSEXCEPTION("Can't place " << cEPId.str());
                }
            }

            m_unNumRobots += unRobots;
            ++teamID; // Increment team ID
        }

        LOG << "[INFO] Number of robots: " << m_unNumRobots << std::endl;
    }
    catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error parsing loop functions!", ex);
    }

    /* Update floor */
    m_pcFloor->SetChanged();

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
        if(Distance(c_position_on_plane, target.first) < target.second) {
            return CColor(191,255,191);
        }
    }
    return CColor::WHITE;
}

/****************************************/
/****************************************/

void CTargetTrackingLoopFunctions::PreStep() {

    LOG << "TIME: " << GetSpace().GetSimulationClock() << std::endl;

    /* Check if any robot is in the target area, and send the location if there is */
    UInt8 teamID; // TEMP: Assume only one team for now

    m_unNumRobotsInTarget = 0;

    CSpace::TMapPerType& m_cEPucks = GetSpace().GetEntitiesByType("e-puck");
    for(CSpace::TMapPerType::iterator itEpuck = m_cEPucks.begin();
        itEpuck != m_cEPucks.end();
        ++itEpuck) {

        CEPuckEntity& cEPuck = *any_cast<CEPuckEntity*>(itEpuck->second);
        CRobot& cController = dynamic_cast<CRobot&>(cEPuck.GetControllableEntity().GetController());
        CVector3 pos3d = cEPuck.GetEmbodiedEntity().GetOriginAnchor().Position;
        CVector2 pos2d = CVector2(pos3d.GetX(), pos3d.GetY());
        teamID = cController.GetTeamID();
        CVector2 targetPos = m_vecTargets[teamID-1].first;
        Real radius = m_vecTargets[teamID-1].second;

        /* Check if the robot is in the target area */
        if((targetPos - pos2d).Length() < radius) {
            ++m_unNumRobotsInTarget;
        }
    }

    /* Set target location to every robot in the team */
    for(CSpace::TMapPerType::iterator itEpuck = m_cEPucks.begin();
        itEpuck != m_cEPucks.end();
        ++itEpuck) {

        CEPuckEntity& cEPuck = *any_cast<CEPuckEntity*>(itEpuck->second);
        CRobot& cController = dynamic_cast<CRobot&>(cEPuck.GetControllableEntity().GetController());

        cController.SetTarget(m_vecTargets[cController.GetTeamID()-1].first, m_vecTargets[cController.GetTeamID()-1].second); // TEMP: Use team assigned target
    }
    
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
