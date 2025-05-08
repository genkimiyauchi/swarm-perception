/*
 * AUTHOR: Genki Miyauchi <g.miyauchi@sheffield.ac.uk>
 *
 * An example controller for obstacle avoidance with the e-puck.
 *
 * The controller uses the proximity sensor to detect obstacles and the
 * wheels to move the robot around.
 *
 * This controller is meant to be used with the XML files:
 *    experiments/target_tracking.argos
 */

#ifndef ROBOT_H
#define ROBOT_H

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
/* Definition of proximity sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_proximity_sensor.h>
/* Definition of the range-and-bearing actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
/* Definition of the range-and-bearing sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
/* Definition of the LEDs actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
/* Definition of the positioning sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
/* Vector2 definitions */
#include <argos3/core/utility/math/vector2.h>
/* Random number generator definitions */
#include <argos3/core/utility/math/rng.h>

/* PID controller */
#include <utility/pid.h>
/* Message structure */
#include <utility/robot_message.h>

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CRobot : public CCI_Controller {

public:

    /*
    * The following variables are used as parameters for
    * turning during navigation. You can set their value
    * in the <parameters> section of the XML configuration
    * file, under the
    * <controllers><footbot_flocking_controller><parameters><wheel_turning>
    * section.
    */
    struct SWheelTurningParams {
        /*
        * The turning mechanism.
        * The robot can be in three different turning states.
        */
        enum ETurningMechanism
        {
            NO_TURN = 0, // go straight
            SOFT_TURN,   // both wheels are turning forwards, but at different speeds
            HARD_TURN    // wheels are turning with opposite speeds
        } TurningMechanism;
        /*
        * Angular thresholds to change turning state.
        */
        CRadians HardTurnOnAngleThreshold;
        CRadians SoftTurnOnAngleThreshold;
        CRadians NoTurnAngleThreshold;
        /* Maximum wheel speed */
        Real MaxSpeed;

        void Init(TConfigurationNode& t_tree);
    };

    /*
    * The following variables are used as parameters for
    * tracking targets. You can set their value
    * in the <parameters> section of the XML configuration
    * file, under the
    * <controllers><leader_controller><parameters><target_tracking>
    * section.
    */
    struct STargetTrackingParams {
        /* Target angle to target in radians */
        Real TargetAngle;
        /* Parameters to be used for PID */
        Real Kp;
        Real Ki;
        Real Kd;
        /* Consider it has arrived to a goal/target if it is within a threshold */
        Real thresRange;
        
        void Init(TConfigurationNode& t_node);
    };

    /*
    * The following variables are used as parameters for
    * flocking interaction. You can set their value
    * in the <parameters> section of the XML configuration
    * file, under the
    * <controllers><leader_controller><parameters><team_flocking>
    * section.
    */
    struct SFlockingInteractionParams {
        /* Target robot-robot distance in cm */
        Real TargetDistanceWalk;   // when exploring
        Real TargetDistanceTarget; // when moving to target
        /* Gain of the Lennard-Jones potential */
        Real Gain;
        /* Exponent of the Lennard-Jones potential */
        Real Exponent;
        /* Switch for whether to flock */
        bool IsFlock;

        void Init(TConfigurationNode& t_node);
        Real GeneralizedLennardJones(Real f_distance);
        Real GeneralizedLennardJonesRepulsionWalk(Real f_distance);
        Real GeneralizedLennardJonesRepulsionTarget(Real f_distance);
    };

    // /* List of move types available to the robot */
    // enum class MoveType {
    //     STOP = 0,    // Stop moving
    //     DIRECT,      // Move directly to a target target
    //     ANGLE_DRIFT, // Move to a target target with an angle drift
    //     WHEEL_DRIFT, // Move to a target target with a wheel drift
    // } currentMoveType;

    /* List of states */
    enum class State {
        RANDOM_WALK = 0,  // Random walk
        BROADCAST_WALK,   // Random walk with broadcast
        BROADCAST_HOMING, // Move to target with broadcast
        IN_TARGET,        // In target with broadcast
    } currentState;

public:

    /* Class constructor. */
    CRobot();

    /* Class destructor. */
    virtual ~CRobot() {}

    /*
    * This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML
    * file in the <controllers><epuck_obstacleavoidance_controller> section.
    */
    virtual void Init(TConfigurationNode& t_node);

    /*
    * This function is called once every time step.
    * The length of the time step is set in the XML file.
    */
    virtual void ControlStep();

    /*
    * This function resets the controller to its state right after the
    * Init().
    * It is called when you press the reset button in the GUI.
    * In this example controller there is no need for resetting anything,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
    virtual void Reset();

    /*
    * Called to cleanup what done by Init() when the experiment finishes.
    * In this example controller there is no need for clean anything up,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
    virtual void Destroy() {}

    /* Set simulation clock tick */
    virtual void SetSecondsPerStep(Real f_seconds_per_step) {
        m_fSecondsPerStep = f_seconds_per_step;
    }

    /* Set team ID */
    virtual void SetTeamID(UInt8 un_team_id) {
        m_unTeamID = un_team_id;
    }

    /* Get team ID */
    virtual UInt8 GetTeamID() const {
        return m_unTeamID;
    }

    /* Get state */
    virtual std::string GetState() const {
        switch(currentState) {
            case State::RANDOM_WALK:
                return "RANDOM_WALK";
            case State::BROADCAST_WALK:
                return "BROADCAST_WALK";
            case State::BROADCAST_HOMING:
                return "BROADCAST_HOMING";
            case State::IN_TARGET:
                return "IN_TARGET";
        }
        return "";
    }

    /* Get whether robot has found the target */
    virtual bool HasFoundTarget() const {
        return m_bTargetFound;
    }

    /* Set target to move towards */
    virtual void SetTarget(const CVector2& c_target, const Real& f_radius) {
        m_cTarget = c_target;
        m_fTargetRadius = f_radius;
    }

protected:

    /*
    * Reset variables
    */
    virtual void ResetVariables();

    /* 
    * Receive messages from neighboring robots.
    */
    virtual void GetMessages();

    /*
    * Gets a direction vector as input and transforms it into wheel actuation.
    */
    void SetWheelSpeedsFromVector(const CVector2& c_heading);
    void SetWheelSpeedsFromVectorHoming(const CVector2& c_heading);

    /*
    * Calculates the vector to the next target.
    */
    virtual CVector2 GetAttractionVector();

    /*
    * Calculate the vector to avoid nearby robots.
    */
    virtual CVector2 GetRobotRepulsionVector(std::vector<Message>& msgs);

    /*
    * Calculate the vector to avoid obstacles.
    */
    virtual CVector2 GetObstacleRepulsionVector();

    /* 
    * Get a flocking vector between itself and the other robots.
    */
    virtual CVector2 GetFlockingVector(std::vector<Message>& msgs);

    /*
    * Run the random walk algorithm.
    */
    virtual CVector2 RandomWalk();

private:

    /* Pointer to the differential steering actuator */
    CCI_DifferentialSteeringActuator* m_pcWheels;
    /* Pointer to the e-puck proximity sensor */
    CCI_ProximitySensor* m_pcProximity;
    /* Pointer to the range-and-bearing actuator */
    CCI_RangeAndBearingActuator* m_pcRABAct;
    /* Pointer to the range-and-bearing sensor */
    CCI_RangeAndBearingSensor* m_pcRABSens;
    /* Pointer to the LEDs actuator */
    CCI_LEDsActuator* m_pcLEDs;
    /* Pointer to the positioning sensor */
    CCI_PositioningSensor* m_pcPosSens;

    /* Random number generator */
    CRandom::CRNG* m_pcRNG;

    /* Simulation clock */
    Real m_fSecondsPerStep;

    /* PID to control the heading angle */
    PID* m_pcPIDHeading;

    /* Outgoing message */
    CByteArray cbyte_msg;

    /* Team ID */
    UInt8 m_unTeamID;

    /* Messages received from nearby robots */
    std::vector<Message> teamMsgs;
    std::vector<Message> otherMsgs;

    /* Target to move towards */
    CVector2 m_cTarget;
    Real m_fTargetRadius;
    Real m_fDistToTarget;
    bool m_bTargetFound;
    bool m_bInTarget;

    /* ### Random walk: parameters ### */
    int m_nRandomWalkTimer;
    Real m_fMinRandomWalkRotationAngle, m_fMaxRandomWalkRotationAngle;
    CVector2 currentRotation;

    /* Broadcast timer */
    int m_nBroadcastTimer;
    Real m_fBroadCastDuration;
    size_t m_unBlinkInterval;
    int m_nBlinkTimer;
    CColor m_cCurrentLEDColor;

    // /* ### ANGLE_DRIFT: parameters ### */
    // /* Angle drift range */
    // CRadians m_cAngleDriftRange;
    // Real m_fMinAngleDrift, m_fMaxAngleDrift;
    // /* Angle drift duration */
    // int m_unAngleDriftDurationTimer;
    // int m_unMinAngleDriftDuration, m_unMaxAngleDriftDuration;

    // /* ### WHEEL_DRIFT: parameters ### */
    // /* Wheel drift duraion */
    // size_t m_unWheelDriftDurationTimer;
    // size_t m_unMinWheelDriftDuration, m_unMaxWheelDriftDuration;
    // /* Wheel drift direction */
    // CVector2 m_cPreviouSumForce;

    /*
    * The following variables are used as parameters for the
    * algorithm. You can set their value in the <parameters> section
    * of the XML configuration file, under the
    * <controllers><epuck_obstacleavoidance_controller> section.
    */
    /* The turning parameters. */
    SWheelTurningParams m_sWheelTurningParams;
    /* The flocking interaction parameters. */
    SFlockingInteractionParams m_sFlockingParams;
    SFlockingInteractionParams m_sBlockingParams;
    /* The target tracking parameters */
    STargetTrackingParams m_sTargetTrackingParams;

};

#endif
