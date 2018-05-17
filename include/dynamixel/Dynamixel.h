#pragma once

typedef unsigned char byte;

#include <map>
#include <vector>
#include <cstring>

#include "SerialPort.h"
#include "Utils.h"

static const int BufferSize = 1024;

class Dynamixel {
public:
    Dynamixel();

    Dynamixel(byte id, SerialPort *port);

    void configure();

    void setDirectionCallback(std::function<void(bool)> callback);

    byte getAddress(std::string address);

    byte getCommand(std::string command);

    int
    sendReceiveCommand(std::string command, std::string address, std::vector<byte> data, std::vector<byte> *outData);

    int formatCommand(byte command, byte address, std::vector<byte>, byte *buffer);

    int getPosition();

    int getCurrentLoad();

    int setGoalPosition(int goal);

    int setMovingSpeed(int speed);

    int getCCWAngleLimit();

    int setCCWAngleLimit(int limit);

    int getCWAngleLimit();

    int setCWAngleLimit(int limit);

private:
    byte _id;
    SerialPort *_port;

    std::function<void(bool)> _callback;
protected:
    std::map<std::string, byte> Addresses;
    std::map<std::string, byte> Commands;
};