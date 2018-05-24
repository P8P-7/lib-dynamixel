#pragma once

typedef unsigned char byte;

#include <vector>

#include "SerialPort.h"
#include "Utils.h"

static const int BufferSize = 1024;

class Dynamixel {
public:
    enum class Commands {
        Get = 2,
        Set = 3,
    };

    enum class Addresses {
        ModelNumber = 0,
        ModelNumberH = 1,
        Version = 2,
        ID = 3,
        BaudRate = 4,
        ReturnDelayTime = 5,
        CWAngleLimit = 6,
        CWAngleLimitH = 7,
        CCWAngleLimit = 8,
        CCWAngleLimitH = 9,
        SystemData2 = 10,
        LimitTemp = 11,
        DownLimitVoltage = 12,
        UpLimitVoltage = 13,
        MaxTorque = 14,
        MaxTorqueH = 15,
        ReturnLevel = 16,
        AlarmLED = 17,
        AlarmShutdown = 18,
        OperatingMode = 19,
        DownCalibration = 20,
        DownCalibrationH = 21,
        UpCalibration = 22,
        UpCalibrationH = 23,
        TorqueStatus = 24,
        LEDStatus = 25,
        CWComplianceMargin = 26,
        CCWComplianceMargin = 27,
        CWComplianceSlope = 28,
        CCWComplianceSlope = 29,
        Goal = 30,
        GoalH = 31,
        MovingSpeed = 32,
        MovingSpeedH = 33,
        TorqueLimit = 34,
        TorqueLimitH = 35,
        Position = 36,
        PositionH = 37,
        PresentSpeed = 38,
        PresentSpeedH = 39,
        Load = 40,
        LoadH = 41,
        Voltage = 42,
        PresentTemperature = 43,
        RegisteredInstruction = 44,
        PauseTime = 45,
        Moving = 46,
        Lock = 47,
        Punch = 48,
        PunchH = 49,
    };

    Dynamixel();

    Dynamixel(byte id, SerialPort &port);

    void configure();

    void setDirectionCallback(std::function<void(bool)> callback);

    byte getAddress(const std::string &address);

    byte getCommand(const std::string &command);

    int sendReceiveCommand(Commands command, Addresses address, std::vector<byte> data, std::vector<byte> *outData);

    int formatCommand(Commands command, Addresses address, std::vector<byte> values, byte *buffer);

    int getPosition();

    int getCurrentLoad();

    int setGoalPosition(short goal);

    int setMovingSpeed(short speed);

    int getCCWAngleLimit();

    int setCCWAngleLimit(short limit);

    int getCWAngleLimit();

    int setCWAngleLimit(short limit);

private:
    byte _id;
    SerialPort &_port;

    std::function<void(bool)> _callback;
};