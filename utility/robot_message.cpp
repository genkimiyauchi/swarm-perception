#include "robot_message.h"

/****************************************/
/****************************************/

Message::Message() {

}

/****************************************/
/****************************************/

Message::Message(CCI_RangeAndBearingSensor::SPacket packet) {

    size_t index = 0;

    /* Core */
    direction = CVector2(packet.Range, packet.HorizontalBearing);
    ID = std::to_string(packet.Data[index++]); // Only stores number part of the id here
    teamID = packet.Data[index++];
    inTarget = packet.Data[index++] == 1 ? true : false;

}

/****************************************/
/****************************************/

Message::~Message() {

}

/****************************************/
/****************************************/

CByteArray Message::GetCByteArray() {

    CByteArray arr = CByteArray(MESSAGE_BYTE_SIZE, 255);
    size_t index = 0;

    /* Sender ID */
    arr[index++] = stoi(ID.substr(2));
    /* Team ID */
    arr[index++] = teamID;
    /* In Target */
    arr[index++] = inTarget ? 1 : 0;

    return arr;
}

/****************************************/
/****************************************/

/* 
* Checks whether the Message is empty or not by checking the direction it was received from
*/
bool Message::Empty() {
    return direction.Length() == 0.0f;
}

/****************************************/
/****************************************/

void Message::Print() const {

    std::cout << "\n##########" << std::endl;
    std::cout << "Direction: " << direction << std::endl;
    std::cout << "ID: " << ID << std::endl;
    std::cout << "Team ID: " << (int)teamID << std::endl;
    std::cout << "In Target: " << (inTarget ? "true" : "false") << std::endl;

}
