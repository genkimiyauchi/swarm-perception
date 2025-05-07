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

    /* Target position */
    UInt8 lsbX = packet.Data[index++]; // least significant byte
    UInt8 msbX = packet.Data[index++]; // most significant byte
    UInt8 lsbY = packet.Data[index++];
    UInt8 msbY = packet.Data[index++];

    int unpackedX = (msbX << 8) | lsbX;
    Real recoveredX = (unpackedX / 1000.0f) - 3.0f; // Assuming the coordinate is in the range of -3.0 to 3.0
    int unpackedY = (msbY << 8) | lsbY;
    Real recoveredY = (unpackedY / 1000.0f) - 3.0f;

    targetPosition = CVector2(recoveredX, recoveredY);
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

    /* Target Position */
    int scaledX = static_cast<int>((targetPosition.GetX() + 3.0f) * 1000.0f);
    int scaledY = static_cast<int>((targetPosition.GetY() + 3.0f) * 1000.0f);

    UInt8 lsbX = scaledX & 0xFF; // least significant byte
    UInt8 msbX = (scaledX >> 8) & 0xFF; // most significant byte
    UInt8 lsbY = scaledY & 0xFF; // least significant byte
    UInt8 msbY = (scaledY >> 8) & 0xFF; // most significant byte

    arr[index++] = lsbX;
    arr[index++] = msbX;
    arr[index++] = lsbY;
    arr[index++] = msbY;

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
    std::cout << "Target Position: " << targetPosition << std::endl;

}
