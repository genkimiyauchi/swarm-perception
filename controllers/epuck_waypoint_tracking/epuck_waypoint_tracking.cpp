#include "epuck_waypoint_tracking.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>

/****************************************/
/****************************************/

void CEPuckWaypointTracking::SWheelTurningParams::Init(TConfigurationNode& t_node) {
   try {
        TurningMechanism = NO_TURN;
        CDegrees cAngle;
        GetNodeAttribute(t_node, "hard_turn_angle_threshold", cAngle);
        HardTurnOnAngleThreshold = ToRadians(cAngle);
        GetNodeAttribute(t_node, "soft_turn_angle_threshold", cAngle);
        SoftTurnOnAngleThreshold = ToRadians(cAngle);
        GetNodeAttribute(t_node, "no_turn_angle_threshold", cAngle);
        NoTurnAngleThreshold = ToRadians(cAngle);
        GetNodeAttribute(t_node, "max_speed", MaxSpeed);
    }
    catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing controller wheel turning parameters.", ex);
    }
}

/****************************************/
/****************************************/

void CEPuckWaypointTracking::SWaypointTrackingParams::Init(TConfigurationNode& t_node) {
    try {
        GetNodeAttribute(t_node, "target_angle", TargetAngle);
        GetNodeAttribute(t_node, "kp", Kp);
        GetNodeAttribute(t_node, "ki", Ki);
        GetNodeAttribute(t_node, "kd", Kd);
        GetNodeAttribute(t_node, "thres_range", thresRange);
    }
    catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing controller waypoint tracking parameters.", ex);
    }
}

/****************************************/
/****************************************/

CEPuckWaypointTracking::CEPuckWaypointTracking() :
    m_pcWheels(NULL),
    m_pcProximity(NULL),
    m_pcRABAct(NULL),
    m_pcRABSens(NULL),
    m_pcLEDs(NULL),
    m_pcPosSens(NULL),
    m_pcRNG(NULL),
    m_pcPIDHeading(NULL) {}

/****************************************/
/****************************************/

void CEPuckWaypointTracking::Init(TConfigurationNode& t_node) {
    /*
    * Get sensor/actuator handles
    *
    * The passed string (ex. "differential_steering") corresponds to the
    * XML tag of the device whose handle we want to have. For a list of
    * allowed values, type at the command prompt:
    *
    * $ argos3 -q actuators
    *
    * to have a list of all the possible actuators, or
    *
    * $ argos3 -q sensors
    *
    * to have a list of all the possible sensors.
    *
    * NOTE: ARGoS creates and initializes actuators and sensors
    * internally, on the basis of the lists provided the configuration
    * file at the <controllers><epuck_obstacleavoidance><actuators> and
    * <controllers><epuck_obstacleavoidance><sensors> sections. If you forgot to
    * list a device in the XML and then you request it here, an error
    * occurs.
    */
    m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    m_pcProximity = GetSensor  <CCI_ProximitySensor             >("proximity"            );
    m_pcRABAct    = GetActuator<CCI_RangeAndBearingActuator     >("range_and_bearing"    );
    m_pcRABSens   = GetSensor  <CCI_RangeAndBearingSensor       >("range_and_bearing"    );
    m_pcLEDs      = GetActuator<CCI_LEDsActuator                >("leds"                 );
    m_pcPosSens   = GetSensor  <CCI_PositioningSensor           >("positioning"          );

    /*
        * Parse the configuration file
        *
        * The user defines this part. Here, the algorithm accepts three
        * parameters and it's nice to put them in the config file so we don't
        * have to recompile if we want to try other settings.
        */
    try {
        /* Wheel turning */
        m_sWheelTurningParams.Init(GetNode(t_node, "wheel_turning"));
        /* Waypoint tracking */
        m_sWaypointTrackingParams.Init(GetNode(t_node, "waypoint_tracking"));
        // /* Flocking-related */
        // m_sFlockingParams.Init(GetNode(t_node, "flocking"));
    }
    catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error parsing the controller parameters.", ex);
    }

    /* Create a new RNG */
    m_pcRNG = CRandom::CreateRNG("argos");

    /* Init PID Controller */
    m_pcPIDHeading = new PID(0.1,       // dt  (loop interval time)
        m_sWheelTurningParams.MaxSpeed,  // max
        -m_sWheelTurningParams.MaxSpeed, // min
        m_sWaypointTrackingParams.Kp,    // Kp
        m_sWaypointTrackingParams.Ki,    // Ki
        m_sWaypointTrackingParams.Kd);   // Kd

    m_cTargetWaypoint.Set(1.0f, 1.0f); // TEMP hard-coded value

    currentMoveType = MoveType::ANGLE_BIAS; // TEMO hard-coded value
}

/****************************************/
/****************************************/

void CEPuckWaypointTracking::ControlStep() {

    // /* Reset variables */
    // ResetVariables();

    // /* Receive new messages */
    // GetMessages();

    /* Get position */
    CVector3 pos3d = m_pcPosSens->GetReading().Position;
    CVector2 pos2d = CVector2(pos3d.GetX(), pos3d.GetY());
    RLOG << "pos: " << pos2d << std::endl;

    /* Attraction to waypoint */
    CVector2 waypointForce = VectorToWaypoint();
    CVector2 sumForce = waypointForce;

    // RLOG << "waypointForce: " << waypointForce << std::endl;

    // sumForce.Normalize();
    // sumForce *= m_sWheelTurningParams.MaxSpeed;
    
    /* Set move type */
    switch(currentMoveType) {
        case MoveType::DIRECT:
            /* Attraction to waypoint */
            sumForce = waypointForce;
            break;
        case MoveType::ANGLE_BIAS:
            if(m_unBiasDuration == 0) {
                /* Choose a random number between 10-30 */
                m_unBiasDuration = m_pcRNG->Uniform(CRange<UInt32>(10, 30)); // 1-3 seconds, TEMP hard-coded value
                /* Choose a random angle between 0-30 degrees in radian */
                m_cAngleBias.FromValueInDegrees(m_pcRNG->Uniform(CRange<Real>(-90, 90))); // TEMP hard-coded value
            }

            RLOG << "rad: " << m_cAngleBias.GetValue() << ", time = " << m_unBiasDuration << std::endl;

            /* Attraction to waypoint */
            sumForce = waypointForce;
            /* Rotate sumForce by m_cAngleBias */
            sumForce.Rotate(m_cAngleBias);
            /* Decrease the duration */
            m_unBiasDuration--;
            
            break;
    }

    SetWheelSpeedsFromVectorHoming(sumForce);

}

/****************************************/
/****************************************/

void CEPuckWaypointTracking::ResetVariables() {
    /* Clear messages received */
    robotMsgs.clear();
}

/****************************************/
/****************************************/

void CEPuckWaypointTracking::GetMessages() {

    /* Get RAB messages from nearby e-pucks */
    const CCI_RangeAndBearingSensor::TReadings& tMsgs = m_pcRABSens->GetReadings();

    if( !tMsgs.empty() ) {
        for(int i = 0; i < tMsgs.size(); i++) {

            Message msg = Message(tMsgs[i]);

            /* Store message */
            robotMsgs.push_back(msg);
        }
    }
}

/****************************************/
/****************************************/

CVector2 CEPuckWaypointTracking::VectorToWaypoint() {
    /* Get current position */
    CVector3 pos3d = m_pcPosSens->GetReading().Position;
    CVector2 pos2d = CVector2(pos3d.GetX(), pos3d.GetY());
    CRadians cZAngle, cYAngle, cXAngle;
    m_pcPosSens->GetReading().Orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);

    /* Calculate a normalized vector that points to the next waypoint */
    CVector2 cAccum = m_cTargetWaypoint - pos2d;

    cAccum.Rotate((-cZAngle).SignedNormalize());

    if(cAccum.Length() > 0.0f) {
        /* Make the vector as long as the max speed */
        cAccum.Normalize();
        cAccum *= m_sWheelTurningParams.MaxSpeed;
    }
    return cAccum;
}

/****************************************/
/****************************************/

CVector2 CEPuckWaypointTracking::GetRobotRepulsionVector(std::vector<Message>& msgs) {

    CVector2 resVec = CVector2();

    // for(size_t i = 0; i < msgs.size(); i++) {
    //     /* Calculate LJ */
    //     Real fLJ = m_sTeamFlockingParams.GeneralizedLennardJonesRepulsion(msgs[i].direction.Length());
    //     /* Sum to accumulator */
    //     resVec += CVector2(fLJ,
    //                         msgs[i].direction.Angle());
    // }

    // /* Calculate the average vector */
    // if( !msgs.empty() )
    //     resVec /= msgs.size();

    // /* Limit the length of the vector to the max speed */
    // if(resVec.Length() > m_sWheelTurningParams.MaxSpeed) {
    //     resVec.Normalize();
    //     resVec *= m_sWheelTurningParams.MaxSpeed * 0.5;
    // }

    return resVec;
}

/****************************************/
/****************************************/

void CEPuckWaypointTracking::SetWheelSpeedsFromVector(const CVector2& c_heading) {
    /* Get the heading angle */
    CRadians cHeadingAngle = c_heading.Angle().SignedNormalize();
    /* Get the length of the heading vector */
    Real fHeadingLength = c_heading.Length();
    /* Clamp the speed so that it's not greater than MaxSpeed */
    Real fBaseAngularWheelSpeed = Min<Real>(fHeadingLength, m_sWheelTurningParams.MaxSpeed);
    /* State transition logic */
    if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::HARD_TURN) {
        if(Abs(cHeadingAngle) <= m_sWheelTurningParams.SoftTurnOnAngleThreshold) {
            m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
        }
    }
    if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::SOFT_TURN) {
        if(Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold) {
            m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
        }
        else if(Abs(cHeadingAngle) <= m_sWheelTurningParams.NoTurnAngleThreshold) {
            m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::NO_TURN;
        }
    }
    if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::NO_TURN) {
        if(Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold) {
            m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
        }
        else if(Abs(cHeadingAngle) > m_sWheelTurningParams.NoTurnAngleThreshold) {
            m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
        }
    }
    /* Wheel speeds based on current turning state */
    Real fSpeed1, fSpeed2;
    switch(m_sWheelTurningParams.TurningMechanism) {
        case SWheelTurningParams::NO_TURN: {
            /* Just go straight */
            fSpeed1 = fBaseAngularWheelSpeed;
            fSpeed2 = fBaseAngularWheelSpeed;
            break;
        }
        case SWheelTurningParams::SOFT_TURN: {
            /* Both wheels go straight, but one is faster than the other */
            Real fSpeedFactor = (m_sWheelTurningParams.HardTurnOnAngleThreshold - Abs(cHeadingAngle)) / m_sWheelTurningParams.HardTurnOnAngleThreshold;
            fSpeed1 = fBaseAngularWheelSpeed - fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
            fSpeed2 = fBaseAngularWheelSpeed + fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
            break;
        }
        case SWheelTurningParams::HARD_TURN: {
            /* Opposite wheel speeds */
            fSpeed1 = -m_sWheelTurningParams.MaxSpeed;
            fSpeed2 =  m_sWheelTurningParams.MaxSpeed;
            break;
        }
    }
    /* Apply the calculated speeds to the appropriate wheels */
    Real fLeftWheelSpeed, fRightWheelSpeed;
    if(cHeadingAngle > CRadians::ZERO) {
        /* Turn Left */
        fLeftWheelSpeed  = fSpeed1;
        fRightWheelSpeed = fSpeed2;
    }
    else {
        /* Turn Right */
        fLeftWheelSpeed  = fSpeed2;
        fRightWheelSpeed = fSpeed1;
    }
    /* Finally, set the wheel speeds */
    m_pcWheels->SetLinearVelocity(fLeftWheelSpeed, fRightWheelSpeed);
}

/****************************************/
/****************************************/

void CEPuckWaypointTracking::SetWheelSpeedsFromVectorHoming(const CVector2& c_heading) {

    /* Get the heading angle */
    CRadians cHeadingAngle = c_heading.Angle().SignedNormalize();
    /* Get the length of the heading vector */
    Real fHeadingLength = c_heading.Length();

    /* Calculate the amount to adjust the wheel speeds */
    Real fSpeed = m_pcPIDHeading->calculate(0,cHeadingAngle.GetValue());

    /* Apply the calculated speeds to the appropriate wheels */
    Real fLeftWheelSpeed, fRightWheelSpeed;
    fLeftWheelSpeed  = m_sWheelTurningParams.MaxSpeed+fSpeed;
    fRightWheelSpeed = m_sWheelTurningParams.MaxSpeed-fSpeed;

    /* Clamp the speed so that it's not greater than MaxSpeed */
    fLeftWheelSpeed = Min<Real>(fLeftWheelSpeed, m_sWheelTurningParams.MaxSpeed);
    fRightWheelSpeed = Min<Real>(fRightWheelSpeed, m_sWheelTurningParams.MaxSpeed);

    /* Finally, set the wheel speeds */
    m_pcWheels->SetLinearVelocity(fLeftWheelSpeed, fRightWheelSpeed);
}

/****************************************/
/****************************************/

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the configuration file to refer to this
 * controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CEPuckWaypointTracking, "epuck_waypoint_tracking_controller")
