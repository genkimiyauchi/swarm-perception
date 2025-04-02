/*
 * AUTHOR: Carlo Pinciroli <cpinciro@ulb.ac.be>
 *
 * An example controller for obstacle avoidance with the e-puck.
 *
 * The controller uses the proximity sensor to detect obstacles and the
 * wheels to move the robot around.
 *
 * This controller is meant to be used with the XML files:
 *    experiments/epuck_waypoint_tracking.argos
 */

#ifndef EPUCK_WAYPOINT_TRACKING_H
#define EPUCK_WAYPOINT_TRACKING_H

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
class CEPuckWaypointTracking : public CCI_Controller {

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
    * tracking waypoints. You can set their value
    * in the <parameters> section of the XML configuration
    * file, under the
    * <controllers><leader_controller><parameters><waypoint_tracking>
    * section.
    */
    struct SWaypointTrackingParams {
        /* Target angle to waypoint in radians */
        Real TargetAngle;
        /* Parameters to be used for PID */
        Real Kp;
        Real Ki;
        Real Kd;
        /* Consider it has arrived to a goal/waypoint if it is within a threshold */
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
        Real TargetDistance;
        /* Gain of the Lennard-Jones potential */
        Real Gain;
        /* Exponent of the Lennard-Jones potential */
        Real Exponent;

        void Init(TConfigurationNode& t_node);
        Real GeneralizedLennardJones(Real f_distance);
        // Real GeneralizedLennardJonesRepulsion(Real f_distance);
    };

    /* List of move types available to the robot */
    enum class MoveType {
        STOP = 0,   // Stop moving
        DIRECT,     // Move directly to a target waypoint
        ANGLE_BIAS, // Move to a target waypoint with an angle bias
    } currentMoveType;

public:

    /* Class constructor. */
    CEPuckWaypointTracking();

    /* Class destructor. */
    virtual ~CEPuckWaypointTracking() {}

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
    * Calculates the vector to the next waypoint.
    */
    virtual CVector2 VectorToWaypoint();

    /* 
    * Get a flocking vector between itself and the other robots.
    */
    virtual CVector2 GetFlockingVector(std::vector<Message>& msgs);

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

    /* PID to control the heading angle */
    PID* m_pcPIDHeading;

    /* Outgoing message */
    CByteArray cbyte_msg;

    /* Messages received from nearby robots */
    std::vector<Message> robotMsgs;

    /* Waypoint to move towards */
    CVector2 m_cTargetWaypoint;

    /* ### ANGLE_BIAS: params ### */
    /* Angle bias */
    CRadians m_cAngleBias;
    /* Duration to apply bias */
    UInt32 m_unBiasDuration;

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
    /* The waypoint tracking parameters */
    SWaypointTrackingParams m_sWaypointTrackingParams;

};

#endif
