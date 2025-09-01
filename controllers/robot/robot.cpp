#include "robot.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>

#include <unordered_map>

/****************************************/
/****************************************/

/* Constants */

// in meters
static const Real BODY_RADIUS = 0.035f;
static const Real INTERWHEEL_DISTANCE = 0.053f; 
static const Real HALF_INTERWHEEL_DISTANCE = INTERWHEEL_DISTANCE * 0.5f;

static const std::vector<CRadians> PROX_ANGLE {
    CRadians::PI / 10.5884f,
    CRadians::PI / 3.5999f,
    CRadians::PI_OVER_TWO,  // side sensor
    CRadians::PI / 1.2f,    // back sensor
    CRadians::PI / 0.8571f, // back sensor
    CRadians::PI / 0.6667f, // side sensor
    CRadians::PI / 0.5806f,
    CRadians::PI / 0.5247f
  };

/* Message target position */
const float OFFSET = 32.0f; // Offset for the target position in meters

// /* Team colors */
// std::unordered_map<UInt8, CColor> teamColor = {
//     {1, CColor::RED},
//     {2, CColor::BLUE},
//     {3, CColor::GREEN},
//     {4, CColor::YELLOW},
//     {5, CColor::ORANGE},
//     {6, CColor::WHITE},
// };

/****************************************/
/****************************************/

void CRobot::SWheelTurningParams::Init(TConfigurationNode& t_node) {
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

void CRobot::SFlockingInteractionParams::Init(TConfigurationNode& t_node) {
    try {
       GetNodeAttribute(t_node, "target_distance_walk", TargetDistanceWalk);
       GetNodeAttribute(t_node, "target_distance_target", TargetDistanceTarget);
       GetNodeAttribute(t_node, "gain", Gain);
       GetNodeAttribute(t_node, "exponent", Exponent);
    }
    catch(CARGoSException& ex) {
       THROW_ARGOSEXCEPTION_NESTED("Error initializing controller flocking parameters.", ex);
    }
 }

/****************************************/
/****************************************/

void CRobot::STargetTrackingParams::Init(TConfigurationNode& t_node) {
    try {
        GetNodeAttribute(t_node, "target_angle", TargetAngle);
        GetNodeAttribute(t_node, "kp", Kp);
        GetNodeAttribute(t_node, "ki", Ki);
        GetNodeAttribute(t_node, "kd", Kd);
        GetNodeAttribute(t_node, "thres_range", thresRange);
    }
    catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing controller target tracking parameters.", ex);
    }
}

/****************************************/
/****************************************/

/*
 * This function is a generalization of the Lennard-Jones potential
 */
Real CRobot::SFlockingInteractionParams::GeneralizedLennardJones(Real f_distance) {
    Real fNormDistExp = ::pow(TargetDistanceWalk / f_distance, Exponent);
    return -Gain / f_distance * (fNormDistExp * fNormDistExp - fNormDistExp);
}

/****************************************/
/****************************************/

/*
 * This function is a generalization of the Lennard-Jones potential for repulsion only 
 */
Real CRobot::SFlockingInteractionParams::GeneralizedLennardJonesRepulsionWalk(Real f_distance) {
    Real fNormDistExp = ::pow(TargetDistanceWalk / f_distance, Exponent);
    return -Gain / f_distance * (fNormDistExp * fNormDistExp);
}

/****************************************/
/****************************************/

/*
 * This function is a generalization of the Lennard-Jones potential for repulsion only 
 */
Real CRobot::SFlockingInteractionParams::GeneralizedLennardJonesRepulsionTarget(Real f_distance) {
    Real fNormDistExp = ::pow(TargetDistanceTarget / f_distance, Exponent);
    return -Gain / f_distance * (fNormDistExp * fNormDistExp);
}

/****************************************/
/****************************************/

CRobot::CRobot() :
    m_pcWheels(NULL),
    m_pcProximity(NULL),
    m_pcRABAct(NULL),
    m_pcRABSens(NULL),
    m_pcLEDs(NULL),
    m_pcPosSens(NULL),
    m_pcRNG(NULL),
    m_pcPIDHeading(NULL),
    m_bSelected(false) {}

/****************************************/
/****************************************/

void CRobot::Init(TConfigurationNode& t_node) {
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
        /* Target tracking */
        m_sTargetTrackingParams.Init(GetNode(t_node, "target_tracking"));
        /* Flocking-related */
        m_sFlockingParams.Init(GetNode(t_node, "flocking"));
        // m_sBlockingParams.Init(GetNode(t_node, "blocking"));
        /* Motion */
        std::string strRandomWalkRotationAngle;
        // std::string strMoveType, strAngleDriftRange, strAngleDriftDuration, strWheelDriftRatio, strWheelDriftDuration, strFlock;
        TConfigurationNode& cMotionNode = GetNode(t_node, "motion");
        // GetNodeAttribute(cMotionNode, "type", strMoveType);
        GetNodeAttribute(cMotionNode, "random_walk_rotation_angle", strRandomWalkRotationAngle);
        GetNodeAttribute(cMotionNode, "broadcast_duration", m_fBroadCastDuration);
        // GetNodeAttributeOrDefault(cMotionNode, "angle_drift_range", strAngleDriftRange, std::string("45,45"));
        // GetNodeAttributeOrDefault(cMotionNode, "angle_drift_duration", strAngleDriftDuration, std::string("10,30"));
        // GetNodeAttributeOrDefault(cMotionNode, "wheel_drift_range", strWheelDriftRatio, std::string("0.5"));
        // GetNodeAttributeOrDefault(cMotionNode, "wheel_drift_duration", strWheelDriftDuration, std::string("10,30"));
        // GetNodeAttribute(cMotionNode, "flock", strFlock);

        // /* Parse move type */
        // if(strMoveType == "direct") {
        //     currentMoveType = MoveType::DIRECT;
        // } 
        // else if(strMoveType == "angle_drift") {
        //     currentMoveType = MoveType::ANGLE_DRIFT;
        // } 
        // else if(strMoveType == "wheel_drift") {
        //     currentMoveType = MoveType::WHEEL_DRIFT;
        // } 
        // else {
        //     THROW_ARGOSEXCEPTION("Invalid move type: " << strMoveType);
        // }

        /* Parse random walk rotation duration */
        std::string::size_type comma_pos = strRandomWalkRotationAngle.find(',');
        if(comma_pos == std::string::npos) {
            THROW_ARGOSEXCEPTION("Invalid random walk duration format: " << strRandomWalkRotationAngle);
        }

        m_fMinRandomWalkRotationAngle = std::stof(strRandomWalkRotationAngle.substr(0, comma_pos));
        m_fMaxRandomWalkRotationAngle = std::stof(strRandomWalkRotationAngle.substr(comma_pos + 1));
        if(m_fMinRandomWalkRotationAngle > m_fMaxRandomWalkRotationAngle) {
            THROW_ARGOSEXCEPTION("Invalid random walk duration range: min > max (" << m_fMinRandomWalkRotationAngle << " > " << m_fMaxRandomWalkRotationAngle << ")");
        }

        // /* Parse angle drift range */
        // comma_pos = strAngleDriftRange.find(',');
        // if(comma_pos == std::string::npos) {
        //     THROW_ARGOSEXCEPTION("Invalid angle drift format: " << strAngleDriftRange);
        // }
        // m_fMinAngleDrift = std::stod(strAngleDriftRange.substr(0, comma_pos));
        // m_fMaxAngleDrift = std::stod(strAngleDriftRange.substr(comma_pos + 1));
        // if(m_fMinAngleDrift > m_fMaxAngleDrift) {
        //     THROW_ARGOSEXCEPTION("Invalid angle drift range: min > max (" << m_fMinAngleDrift << " > " << m_fMaxAngleDrift << ")");
        // }
        
        // /* Parse angle drift duration */
        // comma_pos = strAngleDriftDuration.find(',');
        // if(comma_pos == std::string::npos) {
        //     THROW_ARGOSEXCEPTION("Invalid angle drift duration format: " << strAngleDriftDuration);
        // }
        // m_unMinAngleDriftDuration = std::stoi(strAngleDriftDuration.substr(0, comma_pos));
        // m_unMaxAngleDriftDuration = std::stoi(strAngleDriftDuration.substr(comma_pos + 1));
        // if(m_unMinAngleDriftDuration > m_unMaxAngleDriftDuration) {
        //     THROW_ARGOSEXCEPTION("Invalid angle drift duration range: min > max (" << m_unMinAngleDriftDuration << " > " << m_unMaxAngleDriftDuration << ")");
        // }

        // /* parse wheel drift duration */
        // comma_pos = strWheelDriftDuration.find(',');
        // if(comma_pos == std::string::npos) {
        //     THROW_ARGOSEXCEPTION("Invalid wheel drift duration format: " << strWheelDriftDuration);
        // }
        // m_unMinWheelDriftDuration = std::stoi(strWheelDriftDuration.substr(0, comma_pos));
        // m_unMaxWheelDriftDuration = std::stoi(strWheelDriftDuration.substr(comma_pos + 1));
        // if(m_unMinWheelDriftDuration > m_unMaxWheelDriftDuration) {
        //     THROW_ARGOSEXCEPTION("Invalid wheel drift duration range: min > max (" << m_unMinWheelDriftDuration << " > " << m_unMaxWheelDriftDuration << ")");
        // }

        // /* Parse whether to flock */
        // if(strFlock == "true") {
        //     m_sFlockingParams.IsFlock = true;
        // } else if(strFlock == "false") {
        //     m_sFlockingParams.IsFlock = false;
        // } else {
        //     THROW_ARGOSEXCEPTION("Invalid flock param: " << strFlock);
        // }

        /* Print parsed parameters */
        // LOG << ", Move type: " << strMoveType;
        // if(currentMoveType == MoveType::ANGLE_DRIFT) {
        //     LOG << " range=(" << m_fMinAngleDrift << "," << m_fMaxAngleDrift << ")";
        //     LOG << " t=(" << m_unMinAngleDriftDuration << "," << m_unMaxAngleDriftDuration << ")";
        // } 
        // else if(currentMoveType == MoveType::WHEEL_DRIFT) {
        //     LOG << " t=(" << m_unMinWheelDriftDuration << "," << m_unMaxWheelDriftDuration << ")";
        // }
        // LOG << ", Flock: " << strFlock;
        // LOG << std::endl;
    }
    catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error parsing the controller parameters.", ex);
    }

    // m_unAngleDriftDurationTimer = 0;
    // m_unWheelDriftDurationTimer = 0;

    /* Create a new RNG */
    m_pcRNG = CRandom::CreateRNG("argos");

    /* Init PID Controller */
    m_pcPIDHeading = new PID(0.1,        // dt  (loop interval time)
        m_sWheelTurningParams.MaxSpeed,  // max
        -m_sWheelTurningParams.MaxSpeed, // min
        m_sTargetTrackingParams.Kp,      // Kp
        m_sTargetTrackingParams.Ki,      // Ki
        m_sTargetTrackingParams.Kd);     // Kd

    /* Set initial state */
    currentState = State::RANDOM_WALK;

    /* Set LED color */
    m_cCurrentLEDColor = CColor::BLUE;
    m_pcLEDs->SetAllColors(m_cCurrentLEDColor);

    /* Initialize variables */
    m_bInTarget = false;
    m_bTargetFound = false;
    m_bTargetReceived = false;
    m_nRandomWalkTimer = 0;
    m_nBroadcastTimer = 0;

    m_cTarget = CVector2(OFFSET, OFFSET); // Set default target position to be outside the arena
}

/****************************************/
/****************************************/

void CRobot::Reset() {

    /* Initialize the msg contents to 255 (Reserved for "no event has happened") */
    m_pcRABAct->ClearData();
    cbyte_msg = CByteArray(MESSAGE_BYTE_SIZE, 255);
    m_pcRABAct->SetData(cbyte_msg);
}

/****************************************/
/****************************************/

void CRobot::ControlStep() {

    // LOG << "---------- " << GetId() << " ----------" << std::endl;

    /* Reset variables */
    ResetVariables();

    /* Receive new messages */
    GetMessages();

    /* Get current position */
    CVector3 pos3d = m_pcPosSens->GetReading().Position;
    CVector2 pos2d = CVector2(pos3d.GetX(), pos3d.GetY());
    CRadians cZAngle, cYAngle, cXAngle;
    m_pcPosSens->GetReading().Orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);

    if(m_cTarget != CVector2(OFFSET, OFFSET)) {
        m_fDistToTarget = Distance(pos2d, m_cTarget);
        m_bInTarget = m_fDistToTarget < m_fTargetRadius ? true : false;
    }

    /* Switch states */
    if(currentState == State::RANDOM_WALK) {
        
        m_bTargetReceived = false;
        if(!m_bInTarget) {
            /* Check if neighboring robots have found the target */
            for(const auto& msg : teamMsgs) {
                // if(GetId() == "ep6") {
                    // RLOG << "msg.targetPosition: " << msg.targetPosition << ", m_cTarget: " << m_cTarget << std::endl;
                    // RLOG << "OFFSET: " << CVector2(OFFSET, OFFSET) << ", m_cTarget: " << m_cTarget << std::endl;
                    // RLOG << "Equal? " << (msg.targetPosition == m_cTarget) << std::endl;
                    // RLOG << "distance: " << Distance(msg.targetPosition, m_cTarget) << std::endl;
                // }
                if(msg.targetPosition != CVector2(OFFSET, OFFSET)) { // Check if a target position is set
                    m_bTargetReceived = true;
                    RLOG << "Target received from " << msg.ID << std::endl;
                    break;
                }
            }
        }
        
        /* Switch to BROADCAST_WALK if target found */
        if(m_bInTarget || m_bTargetReceived) {
            currentState = State::BROADCAST_WALK;
            size_t numberOfBlinks = 30; // TEMP: hard-coded value
            m_nBroadcastTimer = (int)(m_fBroadCastDuration / m_fSecondsPerStep);
            // m_unBlinkInterval = (size_t)(m_nBroadcastTimer / (numberOfBlinks * 2));
            m_unBlinkInterval = 5; // TEMP: hard-coded value
            // RLOG << "Broadcasting for " << m_fBroadCastDuration << " seconds" << std::endl;
            // RLOG << "Timer: " << m_nBroadcastTimer << std::endl;
            // RLOG << "Blink interval: " << m_unBlinkInterval << std::endl;
            m_nBlinkTimer = 0;

            m_bTargetFound = true;
        }
    }
    else if(currentState == State::BROADCAST_WALK) {

        /* Broadcast message while walking randomly for a certain duration */
        --m_nBroadcastTimer;

        if(!m_bSelected && m_nBroadcastTimer <= 0) {
            currentState = State::BROADCAST_HOMING;
        } else if(m_bSelected && m_bMoveToTarget) {
            currentState = State::BROADCAST_HOMING;
        }
    }
    else if(currentState == State::BROADCAST_HOMING) {
        
        /* Move towards target */
        if(m_bInTarget) {
            currentState = State::IN_TARGET;
        }
    }
    else if(currentState == State::IN_TARGET) {
        // no switching
    }

    /* Set motion vector */
    CVector2 motionVector;
    if(m_bSelected && !m_bMoveToTarget) {
        // skip
    } else {
        if(currentState == State::RANDOM_WALK || currentState == State::BROADCAST_WALK) {
            /* Random walk */
            motionVector = RandomWalk();
        }
        else if(currentState == State::BROADCAST_HOMING || currentState == State::IN_TARGET) {
            /* Move towards target */

            CVector2 targetForce = GetAttractionVector();

            if(currentState == State::IN_TARGET) {
                /* Reduce attraction from the target center as it gets closer to it */
                targetForce *= m_fDistToTarget / m_fTargetRadius;
            }

            /* Flocking or Repulsion force */
            std::vector<Message> allMsgs;
            allMsgs.insert(allMsgs.end(), teamMsgs.begin(), teamMsgs.end());
            allMsgs.insert(allMsgs.end(), otherMsgs.begin(), otherMsgs.end());
            // CVector2 flockingForce = GetFlockingVector(allMsgs);
            CVector2 repulsionForce = GetRobotRepulsionVector(allMsgs);

            // CVector2 obstacleForce = GetObstacleRepulsionVector();

            /* Sum of forces */
            motionVector = targetForce + repulsionForce;// + obstacleForce * 10;
        }
    }

    /* Set LED color according to its state */
    if(currentState == State::RANDOM_WALK) {
        m_cCurrentLEDColor = CColor::BLUE;
    }
    else if(currentState == State::BROADCAST_WALK) {
        
        if(m_bSelected) {

            /* Controlled by a user */

            if(m_bShareTarget) {
                if(m_nBlinkTimer <= 0) {

                    m_nBlinkTimer = m_unBlinkInterval; // Reset blink timer

                    /* Toggle color */
                    if(m_cCurrentLEDColor == CColor::RED) {
                        m_cCurrentLEDColor = CColor::BLACK;
                    } else {
                        m_cCurrentLEDColor = CColor::RED;
                    }
                } else {
                    --m_nBlinkTimer;
                }
            }

        } else {

            /* Not controlled by a user */

            if(m_nBlinkTimer <= 0) {

                m_nBlinkTimer = m_unBlinkInterval; // Reset blink timer

                /* Toggle color */
                if(m_cCurrentLEDColor == CColor::RED) {
                    m_cCurrentLEDColor = CColor::BLACK;
                } else {
                    m_cCurrentLEDColor = CColor::RED;
                }
            } else {
                --m_nBlinkTimer;
            }
        }
    }
    else if(currentState == State::BROADCAST_HOMING) {
        m_cCurrentLEDColor = CColor::GREEN;
    }
    else if(currentState == State::IN_TARGET) {
        m_cCurrentLEDColor = CColor::GREEN;
    }
    m_pcLEDs->SetAllColors(m_cCurrentLEDColor); // Set LED color

    /* DEBUGGING */
    // RLOG << "targetForce: " << targetForce << std::endl;
    // RLOG << "repulsionForce: " << repulsionForce << std::endl;
    // RLOG << "sumForce: " << sumForce << std::endl;

    // sumForce.Normalize();
    // sumForce *= m_sWheelTurningParams.MaxSpeed;
    
    // /* Set move type */
    // switch(currentMoveType) {
    //     case MoveType::DIRECT:
    //         break;

    //     case MoveType::ANGLE_DRIFT:
    //         if(m_unAngleDriftDurationTimer == 0) {
    //             /* Choose a random duration */
    //             m_unAngleDriftDurationTimer = m_pcRNG->Uniform(CRange<UInt32>(m_unMinAngleDriftDuration, m_unMaxAngleDriftDuration));
    //             /* Choose a random angle */
    //             m_cAngleDriftRange.FromValueInDegrees(m_pcRNG->Uniform(CRange<Real>(m_fMinAngleDrift, m_fMaxAngleDrift)));
    //         }

    //         // RLOG << "rad: " << m_cAngleDriftRange.GetValue() << ", time = " << m_unAngleDriftDurationTimer << std::endl;

    //         /* Rotate sumForce by m_cAngleDriftRange */
    //         motionVector.Rotate(m_cAngleDriftRange);
    //         /* Decrease the duration */
    //         m_unAngleDriftDurationTimer--;
            
    //         break;

    //     // case MoveType::WHEEL_DRIFT:
    //     //     if(m_unWheelDriftDurationTimer == 0) {
    //     //         /* Choose a random duration */
    //     //         m_unWheelDriftDurationTimer = m_pcRNG->Uniform(CRange<UInt32>(m_unMinWheelDriftDuration, m_unMaxWheelDriftDuration));
    //     //     }

    //     //     /* Apply wheel drift */
    //     //     // sumForce = m_cPreviouSumForce;

    //     //     m_unWheelDriftDurationTimer--;

    //     default:
    //         break;
    // }

    // /* TEMP */
    // // go through each message in teamMsgs and Print
    // for(size_t i = 0; i < teamMsgs.size(); i++) {
    //     teamMsgs[i].Print();
    // }

    /* Set Wheel Speed */
    if(m_bSelected && !m_bMoveToTarget) {
        /* Follow the control vector */
        SetWheelSpeedsFromVectorEightDirections(m_cControl);
    }
    else if(motionVector.Length() > m_sWheelTurningParams.MaxSpeed/10) { // motion vector must be at least 10% of the max speed
        SetWheelSpeedsFromVector(motionVector);
    } 
    else {
        m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
    }

    /* Message to broadcast */
    Message msg = Message();
    msg.ID = GetId();
    msg.teamID = m_unTeamID;
    msg.inTarget = m_bInTarget;
    if(currentState == State::BROADCAST_WALK || 
       currentState == State::BROADCAST_HOMING ||
       currentState == State::IN_TARGET) {

        if(m_bSelected) {

            /* Controlled by a user */

            if(m_bShareTarget) {
                msg.targetPosition = m_cTarget;
            }
        } else {
            msg.targetPosition = m_cTarget;
        }
    }

    cbyte_msg = msg.GetCByteArray();
    m_pcRABAct->SetData(cbyte_msg);

}

/****************************************/
/****************************************/

void CRobot::ResetVariables() {
    /* Clear messages received */
    teamMsgs.clear();
    otherMsgs.clear();
}

/****************************************/
/****************************************/

void CRobot::GetMessages() {

    /* Get RAB messages from nearby e-pucks */
    const CCI_RangeAndBearingSensor::TReadings& tMsgs = m_pcRABSens->GetReadings();

    if( !tMsgs.empty() ) {
        for(int i = 0; i < tMsgs.size(); i++) {

            Message msg = Message(tMsgs[i]);

            /* Store message */
            if(msg.teamID == m_unTeamID) {
                teamMsgs.push_back(msg);
            } else {
                otherMsgs.push_back(msg);
            }
        }
    }
}

/****************************************/
/****************************************/

CVector2 CRobot::GetAttractionVector() {
    /* Get current position */
    CVector3 pos3d = m_pcPosSens->GetReading().Position;
    CVector2 pos2d = CVector2(pos3d.GetX(), pos3d.GetY());
    CRadians cZAngle, cYAngle, cXAngle;
    m_pcPosSens->GetReading().Orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);

    CVector2 resVec;

    // /* Move to the edge if it is in the target area */
    // if(DistToTarget < m_fTargetRadius) {

    //     /* No attraction */

    //     // /* Surround target area */
    //     // if(DistToTarget < m_fTargetRadius - BODY_RADIUS) {
    //     //     resVec = pos2d - m_cTarget; // move away from target
    //     // } else {
    //     //     resVec = m_cTarget - pos2d; // move towards target
    //     // }

    //     // resVec.Rotate((-cZAngle).SignedNormalize());
    //     // resVec.Normalize();
    //     // resVec *= Abs((m_fTargetRadius - BODY_RADIUS) - (pos2d - m_cTarget).Length());
    //     // resVec *= 50; // TEMP: hard-coded value

    //     // // Attract to other robots
    //     // CVector2 avgPos = CVector2();
    //     // for(const auto& msg : otherMsgs) {
    //     //     avgPos += CVector2(msg.direction.GetX(), msg.direction.GetY());
    //     // }

    //     // if( !otherMsgs.empty() ) {
    //     //     avgPos /= otherMsgs.size();
    //     //     avgPos.Normalize();
    //     //     avgPos *= 0.1; // TEMP: hard-coded value
    //     // }

    //     // resVec += avgPos;
    //     // // RLOG << "length: " << resVec.Length() << std::endl;

    // } else {
        /* Calculate a normalized vector that points to the next target */
        resVec = m_cTarget - pos2d;
        resVec.Rotate((-cZAngle).SignedNormalize());
        resVec.Normalize();
        resVec *= m_sWheelTurningParams.MaxSpeed;
    // }

    if(resVec.Length() > m_sWheelTurningParams.MaxSpeed) {
        /* Make the vector as long as the max speed */
        resVec.Normalize();
        resVec *= m_sWheelTurningParams.MaxSpeed;
    }

    return resVec;
}

/****************************************/
/****************************************/

CVector2 CRobot::GetRobotRepulsionVector(std::vector<Message>& msgs) {
    
    CVector2 resVec = CVector2();
    size_t counter = 0;

    for(size_t i = 0; i < msgs.size(); i++) {
        /* Calculate LJ */
        Real fLJ;
        if(currentState == State::RANDOM_WALK || currentState == State::BROADCAST_WALK) {
            fLJ = m_sFlockingParams.GeneralizedLennardJonesRepulsionWalk(msgs[i].direction.Length());
        } else if(currentState == State::BROADCAST_HOMING || currentState == State::IN_TARGET) {
            fLJ = m_sFlockingParams.GeneralizedLennardJonesRepulsionTarget(msgs[i].direction.Length());
        }
        /* Sum to accumulator */
        resVec += CVector2(fLJ, msgs[i].direction.Angle());
        /* Count the number of messages */
        ++counter;
    }

    /* Calculate the average vector */
    if(counter > 0)
        resVec /= counter;

    /* Limit the length of the vector to the max speed */
    if(resVec.Length() > m_sWheelTurningParams.MaxSpeed) {
        /* Make the vector as long as the max speed */
        resVec.Normalize();
        resVec *= m_sWheelTurningParams.MaxSpeed;
    }

    return resVec;
}

/****************************************/
/****************************************/

CVector2 CRobot::GetObstacleRepulsionVector() {
    /* Get proximity sensor readings */
    std::vector<Real> fProxReads = m_pcProximity->GetReadings();

    CVector2 resVec = CVector2();

    for(size_t i = 0; i < fProxReads.size(); i++) {
        CVector2 vec = CVector2();
        if(fProxReads[i] > 0.0f) {

            Real distance = -( log(fProxReads[i]) / log(exp(1)) );
            Real length = (0.1 - distance) / 0.1 * m_sWheelTurningParams.MaxSpeed;
            vec = CVector2(length, PROX_ANGLE[i]);
            
            resVec -= vec; // Subtract because we want the vector pointing away from the obstacle
        }
    }

    resVec /= 8; // Number of e-puck sensors

    /* Limit the length of the vector to the max speed */
    if(resVec.Length() > m_sWheelTurningParams.MaxSpeed) {
        resVec.Normalize();
        resVec *= m_sWheelTurningParams.MaxSpeed;
    }

    return resVec;
}

/****************************************/
/****************************************/

CVector2 CRobot::GetFlockingVector(std::vector<Message>& msgs) {

    CVector2 resVec = CVector2();

    /* Get current position */
    CVector3 pos3d = m_pcPosSens->GetReading().Position;
    CVector2 pos2d = CVector2(pos3d.GetX(), pos3d.GetY());
    CRadians cZAngle, cYAngle, cXAngle;
    m_pcPosSens->GetReading().Orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);

    size_t counter = 0;

    for(size_t i = 0; i < msgs.size(); i++) {
        /* Calculate LJ */
        Real fLJ;
        size_t prev_counter = counter;

        if(m_bInTarget) { // Blocking behavior while inside target area

            fLJ = m_sBlockingParams.GeneralizedLennardJonesRepulsionTarget(msgs[i].direction.Length());
            ++counter;

        } else {
            if(msgs[i].inTarget == false) { // Don't flock with or avoid robots in target area
                if(m_sFlockingParams.IsFlock && // Flocking enabled for this team
                    msgs[i].teamID == m_unTeamID) { // Flock with robots in the same team
                     
                    fLJ = m_sFlockingParams.GeneralizedLennardJones(msgs[i].direction.Length());
                    ++counter;
                } else {
                    fLJ = m_sFlockingParams.GeneralizedLennardJonesRepulsionWalk(msgs[i].direction.Length());
                    ++counter;
                }
            }
        }

        /* Sum to accumulator */
        if(counter > prev_counter) {
            resVec += CVector2(fLJ, msgs[i].direction.Angle());
        }
    }

    /* Calculate the average vector */
    if(counter > 0)
        resVec /= counter;

    /* Limit the length of the vector to the max speed */
    if(resVec.Length() > m_sWheelTurningParams.MaxSpeed) {
        resVec.Normalize();
        resVec *= m_sWheelTurningParams.MaxSpeed;
    }

    return resVec;
}

/****************************************/
/****************************************/

CVector2 CRobot::RandomWalk() {

    /* Decrement timer */
    --m_nRandomWalkTimer;

    std::vector<Message> allMsgs;
    allMsgs.insert(allMsgs.end(), teamMsgs.begin(), teamMsgs.end());
    allMsgs.insert(allMsgs.end(), otherMsgs.begin(), otherMsgs.end());

    for(const auto& msg : allMsgs) {
        if(msg.direction.Length() < m_sFlockingParams.TargetDistanceWalk) {
            // RLOG << "Avoiding robots" << std::endl;
            return GetRobotRepulsionVector(allMsgs);
        }
    }

    /* Get proximity sensor readings */
    std::vector<Real> fProxReads = m_pcProximity->GetReadings();

    if(m_nRandomWalkTimer <= 0) {
        Real fDistanceLeft, fDistanceRight;
        Real fLengthLeft = -1;
        Real fLengthRight = -1;
    
        size_t index = 0;
        if(fProxReads[index] > 0.0f) {
            fDistanceLeft = -( log(fProxReads[index]) / log(exp(1)) );
            fLengthLeft = (0.1 - fDistanceLeft) / 0.1;
            // RLOG << "length(" << index << "): " << fLengthLeft << std::endl;
        }
    
        index = 7;
        if(fProxReads[index] > 0.0f) {
            fDistanceRight = -( log(fProxReads[index]) / log(exp(1)) );
            fLengthRight = (0.1 - fDistanceRight) / 0.1;
            // RLOG << "length(" << index << "): " << fLengthRight << std::endl;
        }

        Real proxThres = 0.1; // 10% of the prox range, TEMP: hard-coded value
        if(fLengthLeft > proxThres || fLengthRight > proxThres) {
    
            /* Choose a random rotation duration */

            /* Generate a random angle in degrees */
            Real fRandomAngleDegrees = m_pcRNG->Uniform(CRange<Real>(m_fMinRandomWalkRotationAngle, m_fMaxRandomWalkRotationAngle));
            /* Convert the angle to radians */
            CRadians cRandomAngle = ToRadians(CDegrees(fRandomAngleDegrees));
            /* Calculate the time it takes to rotate */
            Real fArcLength = cRandomAngle.GetValue() * HALF_INTERWHEEL_DISTANCE;
            Real fDuration = fArcLength / (m_sWheelTurningParams.MaxSpeed/100); // convert to from cm -> m
            // RLOG << "fRandomAngleDegrees: " << fRandomAngleDegrees << std::endl;
            // RLOG << "cRandomAngle: " << cRandomAngle.GetValue() << std::endl;
            // RLOG << "fArcLength: " << fArcLength << std::endl;
            // RLOG << "fDuration: " << fDuration << std::endl;
            m_nRandomWalkTimer = (int)(fDuration / m_fSecondsPerStep);
            // RLOG << "fDuration: " << fDuration << std::endl;
            // RLOG << "Random walk duration: " << m_nRandomWalkTimer << std::endl;
    
            if(fLengthLeft < fLengthRight) {
                /* Rotate left */
                // RLOG << "Rotate left" << std::endl;
                currentRotation = CVector2(-m_sWheelTurningParams.MaxSpeed, m_sWheelTurningParams.MaxSpeed);
            } else {
                /* Rotate right */
                // RLOG << "Rotate right" << std::endl;
                currentRotation = CVector2(-m_sWheelTurningParams.MaxSpeed, -m_sWheelTurningParams.MaxSpeed);
            }
        } else {
            /* Move forward */
            // RLOG << "Move forward" << std::endl;
            return CVector2(m_sWheelTurningParams.MaxSpeed, 0);
        }
    }

    // RLOG << "Continue rotation: " << currentRotation << std::endl;
    return currentRotation;
}

/****************************************/
/****************************************/

void CRobot::SetWheelSpeedsFromVector(const CVector2& c_heading) {
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

void CRobot::SetWheelSpeedsFromVectorHoming(const CVector2& c_heading) {

    /* Get the heading angle */
    CRadians cHeadingAngle = c_heading.Angle().SignedNormalize();
    /* Get the length of the heading vector */
    Real fHeadingLength = c_heading.Length();

    /* Calculate the amount to adjust the wheel speeds */
    Real fSpeed = m_pcPIDHeading->calculate(0,cHeadingAngle.GetValue());

    /* Apply the calculated speeds to the appropriate wheels */
    Real fLeftWheelSpeed, fRightWheelSpeed;
    fLeftWheelSpeed  = m_sWheelTurningParams.MaxSpeed - abs(fSpeed) + fSpeed;
    fRightWheelSpeed = m_sWheelTurningParams.MaxSpeed - abs(fSpeed) - fSpeed;
    RLOG << "fSpeed " << fSpeed << ", abs " << abs(fSpeed) << ", left " << fLeftWheelSpeed << ", right " << fRightWheelSpeed << std::endl;

    /* Clamp the speed so that it's not greater than MaxSpeed */
    fLeftWheelSpeed = Min<Real>(fLeftWheelSpeed, m_sWheelTurningParams.MaxSpeed);
    fRightWheelSpeed = Min<Real>(fRightWheelSpeed, m_sWheelTurningParams.MaxSpeed);

    /* Finally, set the wheel speeds */
    m_pcWheels->SetLinearVelocity(fLeftWheelSpeed, fRightWheelSpeed);
}

/****************************************/
/****************************************/

void CRobot::SetWheelSpeedsFromVectorEightDirections(const CVector2& c_heading) {
    /* Get the heading angle */
    CRadians cHeadingAngle = c_heading.Angle().SignedNormalize();
    /* Get the length of the heading vector */
    Real fHeadingLength = c_heading.Length();
    /* Clamp the speed so that it's not greater than MaxSpeed */
    Real fBaseAngularWheelSpeed = Min<Real>(fHeadingLength, m_sWheelTurningParams.MaxSpeed);

    /* Wheel speeds based on current turning state */
    Real fLeftWheelSpeed = 0;
    Real fRightWheelSpeed = 0;

    if(c_heading.GetX() > 0) {
        if(c_heading.GetY() > 0) {
            fLeftWheelSpeed  = fBaseAngularWheelSpeed / 2;
            fRightWheelSpeed = fBaseAngularWheelSpeed;
        } else if(c_heading.GetY() < 0) {
            fLeftWheelSpeed  = fBaseAngularWheelSpeed;
            fRightWheelSpeed = fBaseAngularWheelSpeed / 2;
        } else {
            fLeftWheelSpeed  = fBaseAngularWheelSpeed;
            fRightWheelSpeed = fBaseAngularWheelSpeed;
        }
    } else if(c_heading.GetX() < 0) {
        if(c_heading.GetY() > 0) {
            fLeftWheelSpeed  = -fBaseAngularWheelSpeed / 2;
            fRightWheelSpeed = -fBaseAngularWheelSpeed;
        } else if(c_heading.GetY() < 0) {
            fLeftWheelSpeed  = -fBaseAngularWheelSpeed;
            fRightWheelSpeed = -fBaseAngularWheelSpeed / 2;
        } else {
            fLeftWheelSpeed  = -fBaseAngularWheelSpeed;
            fRightWheelSpeed = -fBaseAngularWheelSpeed;
        }
    } else if(c_heading.GetX() == 0) {
        if(c_heading.GetY() > 0) {
            fLeftWheelSpeed  = -fBaseAngularWheelSpeed;
            fRightWheelSpeed = fBaseAngularWheelSpeed;
        } else if(c_heading.GetY() < 0) {
            fLeftWheelSpeed  = fBaseAngularWheelSpeed;
            fRightWheelSpeed = -fBaseAngularWheelSpeed;
        }
    }

    RLOG << "fLeftWheelSpeed: " << fLeftWheelSpeed << ", fRightWheelSpeed: " << fRightWheelSpeed << std::endl;

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
REGISTER_CONTROLLER(CRobot, "robot_controller")
