#include <cstdio>
#include <cstring>

#include "dynamixel/Dynamixel.h"

//
// Base Dynamixel Class
//
Dynamixel::Dynamixel() = default;

Dynamixel::Dynamixel(byte id, SerialPort *port)
        : _id(id),
          _port(port) {
}

void Dynamixel::setDirectionCallback(std::function<void(std::string)> callback) {
    _callback = std::move(callback);
}

void Dynamixel::configure() {
    Commands["Get"] = 2;
    Commands["Set"] = 3;

    Addresses["ModelNumber"] = 0;
    Addresses["ModelNumberH"] = 1;
    Addresses["Version"] = 2;
    Addresses["ID"] = 3;
    Addresses["BaudRate"] = 4;
    Addresses["ReturnDelayTime"] = 5;
    Addresses["CWAngleLimit"] = 6;
    Addresses["CWAngleLimitH"] = 7;
    Addresses["CCWAngleLimit"] = 8;
    Addresses["CCWAngleLimitH"] = 9;
    Addresses["SystemData2"] = 10;
    Addresses["LimitTemp"] = 11;
    Addresses["DownLimitVoltage"] = 12;
    Addresses["UpLimitVoltage"] = 13;
    Addresses["MaxTorque"] = 14;
    Addresses["MaxTorqueH"] = 15;
    Addresses["ReturnLevel"] = 16;
    Addresses["AlarmLED"] = 17;
    Addresses["AlarmShutdown"] = 18;
    Addresses["OperatingMode"] = 19;
    Addresses["DownCalibration"] = 20;
    Addresses["DownCalibrationH"] = 21;
    Addresses["UpCalibration"] = 22;
    Addresses["UpCalibrationH"] = 23;
    Addresses["TorqueStatus"] = 24;
    Addresses["LEDStatus"] = 25;
    Addresses["CWCompMargin"] = 26;
    Addresses["CCWCompMargin"] = 27;
    Addresses["CWCompSlope"] = 28;
    Addresses["CCWCompSlope"] = 29;
    Addresses["Goal"] = 30;
    Addresses["GoalH"] = 31;
    Addresses["MovingSpeed"] = 32;
    Addresses["MovingSpeedH"] = 33;
    Addresses["TorqueLimit"] = 34;
    Addresses["TorqueLimitH"] = 35;
    Addresses["Position"] = 36;
    Addresses["PositionH"] = 37;
    Addresses["PresentSpeed"] = 38;
    Addresses["PresentSpeedH"] = 39;
    Addresses["Load"] = 40;
    Addresses["LoadH"] = 41;
    Addresses["Voltage"] = 42;
    Addresses["PresentTemperature"] = 43;
    Addresses["RegisteredInstruction"] = 44;
    Addresses["PauseTime"] = 45;
    Addresses["Moving"] = 46;
    Addresses["Lock"] = 47;
    Addresses["Punch"] = 48;
    Addresses["PunchH"] = 49;
}

byte Dynamixel::getAddress(std::string address) {
    return Addresses[address];
}

byte Dynamixel::getCommand(std::string command) {
    return Commands[command];
}

int Dynamixel::sendReceiveCommand(std::string command, std::string address,
                                  std::vector<byte> data,
                                  std::vector<byte> *outData) {
    byte sendBuf[BufferSize] = {0};
    byte recvBuf[BufferSize] = {0};
    int responseLength = 6;
    if (command == "Get") {
        responseLength += data[0];
    }
    int length = formatCommand(Commands[command], Addresses[address], data, sendBuf);

    if (_callback) {
        _callback("tx");
    }

    // send
    long l = _port->sendArray(sendBuf, length);

    if (_callback) {
        _callback("rx");
    }

    // recv 1 (omitted)

    // recv 2
    int recvLen;
    recvLen = _port->getArray(recvBuf, responseLength); // receive again to get the real data

    // check data
    if (recvLen >= responseLength) {
        int numValues = responseLength - 6;
        for (int i = 0; i < numValues; i++) {
            outData->push_back(recvBuf[5 + i]);
        }
        return recvBuf[4]; // the error code if there is one
    } else {
        return -1;
    }
}

int Dynamixel::formatCommand(byte command, byte address, std::vector<byte> values, byte *buffer) {
    byte numberOfParameters = 0;

    //OXFF 0XFF ID LENGTH INSTRUCTION PARAMETER1 PARAMETER N CHECK_SUM
    buffer[0] = 0xff;
    buffer[1] = 0xff;
    buffer[2] = _id;

    // bodyLength
    buffer[3] = 0; // temp

    //the instruction
    buffer[4] = command;

    // start of goal registers
    buffer[5] = address;

    //bytes to write
    for (int i = 0; i < values.size(); i++) {
        buffer[6 + i] = values[i];
    }
    numberOfParameters = values.size();

    // bodyLength
    buffer[3] = (byte) (numberOfParameters + 3);

    byte checksum = Utils::checkSum(buffer, 6 + numberOfParameters);
    buffer[6 + numberOfParameters] = checksum;

    return 7 + numberOfParameters;
}

int Dynamixel::getPosition() {
    std::vector<byte> data = {2}; // number of bytes to read
    std::vector<byte> returnData;
    sendReceiveCommand("Get", "Position", data, &returnData);
    if (returnData.size() == 2) {
        return Utils::convertFromHL(returnData[0], returnData[1]);
    }
    return -1;
}

int Dynamixel::setGoalPosition(int goal) {
    byte posH, posL;
    Utils::convertToHL(goal, &posH, &posL);
    std::vector<byte> data = {posL, posH};
    std::vector<byte> returnData;
    return sendReceiveCommand("Set", "Goal", data, &returnData);
}

int Dynamixel::setMovingSpeed(int speed) {
    byte speedH, speedL;
    Utils::convertToHL(speed, &speedH, &speedL);
    std::vector<byte> data = {speedL, speedH};
    std::vector<byte> returnData;
    return sendReceiveCommand("Set", "MovingSpeed", data, &returnData);
}

int Dynamixel::setCWAngleLimit(int limit) {
    byte limitH, limitL;
    Utils::convertToHL(limit, &limitH, &limitL);
    std::vector<byte> data = {limitL, limitH};
    std::vector<byte> returnData;
    return sendReceiveCommand("Set", "CWAngleLimit", data, &returnData);
}

int Dynamixel::setCCWAngleLimit(int limit) {
    byte limitH, limitL;
    Utils::convertToHL(limit, &limitH, &limitL);
    std::vector<byte> data = {limitL, limitH};
    std::vector<byte> returnData;
    return sendReceiveCommand("Set", "CCWAngleLimit", data, &returnData);
}

int Dynamixel::setWheelMode() {
    setCWAngleLimit(0);
    setCCWAngleLimit(0);
}

//
// AX12
//
AX12::AX12()
        : Dynamixel() {
}

AX12::AX12(byte id, SerialPort *port)
        : Dynamixel(id, port) {
}

void AX12::configure() {
    Dynamixel::configure();
    Addresses["CCWComplianceSlope"] = 29;
    Addresses["CWComplianceSlope"] = 28;
    Addresses["CCWComplianceMargin"] = 27;
    Addresses["CWComplianceMargin"] = 26;
}

float AX12::posToAngle(short pos) {
    float angle = 0;
    angle = (float) pos * 0.29f;
    return angle;
}

short AX12::angleToPos(float angle) {
    short pos = 0;
    pos = (short) (angle / 0.29f);
    return pos;
}

int AX12::setCWComplianceMargin(byte margin) {
    std::vector<byte> data = {margin};
    std::vector<byte> returnData;
    return sendReceiveCommand("Set", "CWComplianceMargin", data, &returnData);
}

int AX12::setCCWComplianceMargin(byte margin) {
    std::vector<byte> data = {margin};
    std::vector<byte> returnData;
    return sendReceiveCommand("Set", "CCWComplianceMargin", data, &returnData);
}

int AX12::setCWComplianceSlope(byte slope) {
    std::vector<byte> data = {slope};
    std::vector<byte> returnData;
    return sendReceiveCommand("Set", "CWComplianceSlope", data, &returnData);
}

int AX12::setCCWComplianceSlope(byte slope) {
    std::vector<byte> data = {slope};
    std::vector<byte> returnData;
    return sendReceiveCommand("Set", "CCWComplianceSlope", data, &returnData);
}
