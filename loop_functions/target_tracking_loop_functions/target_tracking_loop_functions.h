#ifndef TARGET_TRACKING_LOOP_FUNCTIONS_H
#define TARGET_TRACKING_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/rng.h>

using namespace argos;

class CTargetTrackingLoopFunctions : public CLoopFunctions {

public:

    CTargetTrackingLoopFunctions();
    virtual ~CTargetTrackingLoopFunctions() {}

    virtual void Init(TConfigurationNode& t_tree);
    
    virtual void Reset();

    virtual void Destroy();

    virtual CColor GetFloorColor(const CVector2& c_position_on_plane);

    virtual void PreStep();

    virtual void PostStep();

    virtual bool IsDrawRobotLabel() const {
        return m_bDrawRobotLabel;
    }

private:

    TConfigurationNode config;

    CFloorEntity* m_pcFloor;
    CRandom::CRNG* m_pcRNG;

    std::vector<std::pair<CVector2, Real>> m_vecTargets;

    /* Frame Grabbing */
    bool m_bFrameGrabbing;
    UInt32 m_unCameraIndex;

    /* Draw configurations */
    bool m_bDrawRobotLabel;

    std::string m_strOutput;
    std::ofstream m_cOutput;

};

#endif
