#include <cstdio>
#include <cstring>
#include <iostream>

#include "dynamixel/Dynamixel.h"

Dynamixel::Dynamixel(byte id, SerialPort &port)
        : _id(id),
          _port(port) {
}

void Dynamixel::setDirectionCallback(std::function<void(bool)> callback) {
    _callback = std::move(callback);
}

int Dynamixel::sendReceiveCommand(Commands command, Addresses address,
                                  std::vector<byte> data,
                                  std::vector<byte> *outData) {
    byte sendBuf[BufferSize] = {0};
    byte recvBuf[BufferSize] = {0};
    int responseLength = 6;
    if (command == Commands::Get) {
        responseLength += data[0];
    }

    int length = formatCommand(command, address, data, sendBuf);
    if (_callback) {
        _callback(true);
    }

    // send
    _port.sendArray(sendBuf, length);
    if (_callback) {
        _callback(false);
    }

    // recv 1 (omitted)

    // recv 2
    int recvLen = _port.getArray(recvBuf, responseLength); // receive again to get the real data

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

int Dynamixel::formatCommand(Commands command, Addresses address, std::vector<byte> values, byte *buffer) {
    byte numberOfParameters = 0;

    //OXFF 0XFF ID LENGTH INSTRUCTION PARAMETER1 PARAMETER N CHECK_SUM
    buffer[0] = 0xff;
    buffer[1] = 0xff;
    buffer[2] = _id;

    // bodyLength
    buffer[3] = 0; // temp

    //the instruction
    buffer[4] = static_cast<byte>(command);

    // start of goal registers
    buffer[5] = static_cast<byte>(address);

    // bytes to write
    for (byte i = 0; i < values.size(); i++) {
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
    byte posH = 0, posL = 0;
    std::vector<byte> data = {posL, posH};
    std::vector<byte> returnData;
    return sendReceiveCommand(Commands::Get, Addresses::Position, data, &returnData);
}

int Dynamixel::getCurrentLoad() {
    byte loadH = 0, loadL = 0;
    std::vector<byte> data = {loadL, loadH};
    std::vector<byte> returnData;
    return sendReceiveCommand(Commands::Get, Addresses::Load, data, &returnData);
}

int Dynamixel::setGoalPosition(short goal) {
    byte posH, posL;
    Utils::convertToHL(goal, &posH, &posL);
    std::vector<byte> data = {posL, posH};
    std::vector<byte> returnData;
    return sendReceiveCommand(Commands::Set, Addresses::Goal, data, &returnData);
}

int Dynamixel::setMovingSpeed(short speed) {
    byte speedH, speedL;
    Utils::convertToHL(speed, &speedH, &speedL);
    std::vector<byte> data = {speedL, speedH};
    std::vector<byte> returnData;
    return sendReceiveCommand(Commands::Set, Addresses::MovingSpeed, data, &returnData);
}

int Dynamixel::getCWAngleLimit() {
    byte limitH = 0, limitL = 0;
    std::vector<byte> data = {limitL, limitH};
    std::vector<byte> returnData;
    return sendReceiveCommand(Commands::Get, Addresses::CWAngleLimit, data, &returnData);
}

int Dynamixel::setCWAngleLimit(short limit) {
    byte limitH, limitL;
    Utils::convertToHL(limit, &limitH, &limitL);
    std::vector<byte> data = {limitL, limitH};
    std::vector<byte> returnData;
    return sendReceiveCommand(Commands::Set, Addresses::CWAngleLimit, data, &returnData);
}

int Dynamixel::getCCWAngleLimit() {
    byte limitH = 0, limitL = 0;
    std::vector<byte> data = {limitL, limitH};
    std::vector<byte> returnData;
    return sendReceiveCommand(Commands::Get, Addresses::CWAngleLimit, data, &returnData);
}

int Dynamixel::setCCWAngleLimit(short limit) {
    byte limitH, limitL;
    Utils::convertToHL(limit, &limitH, &limitL);
    std::vector<byte> data = {limitL, limitH};
    std::vector<byte> returnData;
    return sendReceiveCommand(Commands::Set, Addresses::CCWAngleLimit, data, &returnData);
}
