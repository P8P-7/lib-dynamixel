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

    void setDirectionCallback(std::function<void(std::string)> callback);

    void setSerialFeedback(bool fb);

    byte getAddress(std::string address);

    byte getCommand(std::string command);

    int sendCommand();

    int receiveCommand();

    int
    sendReceiveCommand(std::string command, std::string address, std::vector<byte> data, std::vector<byte> *outData);

    int formatCommand(byte command, byte address, std::vector<byte>, byte *buffer);

    int setID(byte id);

    int setBaudRate(byte baudRate);

    int getPosition();

    int setGoalPosition(int goal);

    int setMovingSpeed(int speed);

    int setCCWAngleLimit(int limit);

    int setCWAngleLimit(int limit);

    int setWheelMode();

private:

    byte _id;
    SerialPort *_port;

    std::function<void(std::string)> _callback;

protected:
    std::map<std::string, byte> Addresses;
    std::map<std::string, byte> Commands;
};

class AX12 : public Dynamixel {
public:
    AX12();

    AX12(byte id, SerialPort *port);

    void configure();

    static float posToAngle(short pos);

    static short angleToPos(float angle);

    int setCCWComplianceMargin(byte margin);

    int setCWComplianceMargin(byte margin);

    int setCCWComplianceSlope(byte slope);

    int setCWComplianceSlope(byte slope);
};

class MX28 : public Dynamixel {
public:
    MX28();

    MX28(byte id, SerialPort *port);

    void configure();

    static float posToAngle(short pos);

    static short angleToPos(float angle);

    int setPGain(byte pGain);

    int setIGain(byte iGain);

    int setDGain(byte dGain);
};
