/*
* AUTHOR: Genki Miyauchi <g.miyauchi@sheffield.ac.uk>
* 
* Define the message structure used to communicate between the robots.
*/

#ifndef ROBOT_MESSAGE_H
#define ROBOT_MESSAGE_H

/*
 * Include some necessary headers.
 */

/* Definition of the CVector2 datatype */
#include <argos3/core/utility/math/vector2.h>

/* Definition of the range-and-bearing actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
/* Definition of the range-and-bearing sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
* Communication buffer size
**
* size = fixed data + End;
*      = 2         + 1
*      = 3
*/
static const UInt32 MESSAGE_BYTE_SIZE = 
    /* fixed data */
    2 + 
    /* End */
    1;


/* 
* Structure to store incoming data received from other robots 
*/
class Message {

    public:

        /* Class constructor */
        Message();

        Message(CCI_RangeAndBearingSensor::SPacket packet);

        virtual ~Message();

        virtual CByteArray GetCByteArray();

        virtual bool Empty();

        virtual void Print() const;

    public:

        /* Core */
        CVector2 direction = CVector2();
        std::string ID;

};

#endif