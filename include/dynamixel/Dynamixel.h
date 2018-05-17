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

    void setSerialFeedback(bool fb);

    byte getAddress(std::string address);

    byte getCommand(std::string command);

    int sendCommand();

    int receiveCommand();

    int
    sendReceiveCommand(std::string command, std::string address, std::vector<byte> data, std::vector<byte> *outData);

    void turn(int speed, std::string direction);

    void turn(int speed, bool direction);

    int formatCommand(byte command, byte address, std::vector<byte>, byte *buffer);

    int setID(byte id);

    int setBaudRate(byte baudRate);

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