#pragma once

#include "SerialPort.h"
#include "Utils.h"

namespace goliath::dynamixel {
    class Dynamixel {
    public:
        using byte = unsigned char;

        // The instruction set.
        // (see the official Dynamixel AX-12 User's manual p.19)
        enum class Instruction : byte {
            Ping = 1,
            Read = 2,
            Write = 3,
            RegWrite = 4,
            Action = 5,
            Reset = 6,
            SyncWrite = 83
        };

        // Control table (addresses).
        // (see the official Dynamixel AX-12 User's manual p.12)
        enum class Address : byte {
            ModelNumber = 0,
            ModelNumberH = 1,
            FirmwareVersion = 2,
            ID = 3,
            BaudRate = 4,
            ReturnDelayTime = 5,
            CWAngleLimit = 6,
            CWAngleLimitH = 7,
            CCWAngleLimit = 8,
            CCWAngleLimitH = 9,
            SystemData2 = 10,
            HighestLimitTemperature = 11,
            LowestLimitVoltage = 12,
            HighestLimitVoltage = 13,
            MaxTorque = 14,
            MaxTorqueH = 15,
            StatusReturnLevel = 16,
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
            GoalPosition = 30,
            GoalPositionH = 31,
            MovingSpeed = 32,
            MovingSpeedH = 33,
            TorqueLimit = 34,
            TorqueLimitH = 35,
            PresentPosition = 36,
            PresentPositionH = 37,
            PresentSpeed = 38,
            PresentSpeedH = 39,
            PresentLoad = 40,
            PresentLoadH = 41,
            PresentVoltage = 42,
            PresentTemperature = 43,
            RegisteredInstruction = 44,
            PauseTime = 45,
            Moving = 46,
            Lock = 47,
            Punch = 48,
            PunchH = 49,
        };

        enum class Error : byte {
            InputVoltage = 1,
            AngleLimit = 2,
            Overheat = 4,
            Range = 8,
            Checksum = 16,
            Overload = 32,
            Instruction = 64,
        };

        /**
         * Construct a Dynamixel actuator class.
         * @param id the unique ID of a Dynamixel unit. It must be in range (0, 0xFD).
         * @param port the serial port where the Dynamixel unit is connected with.
         */
        Dynamixel(byte id, std::shared_ptr<SerialPort> port);

        /* Low level functions */

        /**
         * Between sending a instruction packet and receiving the status packet we need
         * to set the GPIO Pin to TX.
         * @param callback that will set the GPIO pin to the appropriate RX/TX.
         */
        void setDirectionCallback(std::function<void(bool)> callback);

        /**
         * Send an instruction packet.
         * @param instruction the instruction for the Dynamixel actuator to perform.
         * @param data a vector of bytes containing the packet's data: the
         * instruction to perform or the status of the Dynamixel actuator.
         * @return the number of bytes written.
         */
        size_t send(Instruction instruction, const std::vector<byte> &data);

        /**
         * Read the status packet.
         * @return a vector of bytes containing the status packet's data.
         */
        std::vector<byte> read();

        /**
         * The "instruction packet" is the packet sent to the Dynamixel units.
         * @param instruction the instruction for the Dynamixel actuator to perform.
         * @param data a vector of bytes containing the packet's data: the
         * instruction to perform or the status of the Dynamixel actuator.
         * @return a vector of bytes containing the instruction packet's data.
         */
        std::vector<byte> getInstructionPacket(Instruction instruction, const std::vector<byte> &data);

        /* High level functions */

        /**
         * Read data from the control table of the specified Dynamixel unit.
         * @param address the starting address of the location where the data
         * is to be read.
         * @param data the length of the data to be read.
         * @return the data that has been read.
         */
        int readData(Address address, size_t length);

        /**
         * Write bytes to the control table of the specified Dynamixel unit.
         * @param address the starting address of the location where the data
         * is to be written.
         * @param data the bytes of the data to be written.
         */
        void writeData(Address address, const std::vector<byte> &data);

        /**
         * Ping the specified Dynamixel unit.
         * @return true if the specified unit is available; otherwise, false.
         */
        bool ping();

        /* High level accessors */

        /**
         * @return the firmware version of the specified Dynamixel unit.
         */
        int getFirmwareVersion();

        /**
         * @return the communication speed (baud rate) of the specified
         * Dynamixel unit.
         */
        unsigned int getBaudRate();

        /**
         * The return delay time is the time it takes (in uSec) for the status
         * packet to return after the instruction packet is sent.
         * @return the return delay time of the specified Dynamixel unit.
         */
        int getReturnDelayTime();

        /**
         * The goal position should be higher or equal than this value, otherwise
         * the *Angle Limit Error Bit* (the second error bit of status packets)
         * will be set to `1`.
         * @return the *clockwise angle limit* of the specified Dynamixel unit.
         */
        int getCWAngleLimit();

        /**
         * The goal position should be lower or equal than this value, otherwise
         * the *Angle Limit Error Bit* (the second error bit of status packets)
         * will be set to `1`.
         * @return the *counter clockwise angle limit* of the specified
         * Dynamixel unit.
         */
        int getCCWAngleLimit();

        /**
         * If the internal temperature of the Dynamixel actuator gets higher than
         * this value, the *Over Heating Error Bit* (the third error bit of
         * status packets) will be set to `1`.
         * The values are in degrees Celsius.
         * @return the maximum tolerated internal temperature for the specified
         * Dynamixel unit.
         */
        int getMaxTemperature();

        /**
         * If the present voltage of the Dynamixel actuator gets lower than
         * this value, the *Voltage Range Error Bit* (the first error bit of
         * status packets) will be set to `1`.
         * The values are in Volts.
         * @return the minimum tolerated operating voltage for the specified
         * Dynamixel unit.
         */
        int getMinVoltage();

        /**
         * If the present voltage of the Dynamixel actuator gets higher than
         * this value, the *Voltage Range Error Bit* (the first error bit of
         * status packets) will be set to `1`.
         * The values are in Volts.
         * @return the maximum tolerated operating voltage for the specified
         * Dynamixel unit.
         */
        int getMaxVoltage();

        /**
         * This value, written in EEPROM, is copied to the *torque limit* bytes
         * (in RAM) when the power is turned ON. Thus, *max torque* is just an
         * initialization value for the actual *torque limit*.
         * If this value is equal to `0`, the Dynamixel unit is configured in
         * *free run mode*.
         *
         * @return the initial maximum torque output of the specified Dynamixel
         * unit.
         */
        int getMaxTorque();

        /**
         * +----------------+----------------------------------------+
         * | Returned value | Meaning                                |
         * +================+========================================+
         * | 0              | Do not respond to any instructions     |
         * +----------------+----------------------------------------+
         * | 1              | Respond only to READ_DATA instructions |
         * +----------------+----------------------------------------+
         * | 2              | Respond to all instructions            |
         * +----------------+----------------------------------------+
         * @return whether the specified Dynamixel unit is configured to return a
         * *status packet* after receiving an *instruction packet*.
         */
        int getStatusReturnLevel();

        /**
         * The calibration value is used to compensate the differences between the
         * potentiometers used in the Dynamixel units.
         * @return the "down calibration" value of the specified Dynamixel unit.
         */
        int getDownCalibration();

        /**
         * The calibration value is used to compensate the differences between the
         * potentiometers used in the Dynamixel units.
         * @return the "up calibration" value of the specified Dynamixel unit.
         */
        int getUpCalibration();

        /**
         * @return true if the torque of the specified Dynamixel unit is
         * enabled; otherwise, false.
         */
        bool isTorqueEnabled();

        /**
         * @return true if the LED of the specified Dynamixel unit is ON
         * otherwise, false.
         */
        bool isLedEnabled();

        /**
         * The compliance feature can be utilized for absorbing shocks at the
         * output shaft.
         * @return the clockwise compliance margin of the specified Dynamixel
         * unit.
         */
        int getCWComplianceMargin();

        /**
         * The compliance feature can be utilized for absorbing shocks at the
         * output shaft.
         * @return the counter clockwise compliance margin of the specified
         * Dynamixel unit.
         */
        int getCCWComplianceMargin();

        /**
         * The compliance feature can be utilized for absorbing shocks at the
         * output shaft.
         * @return the clockwise compliance slope of the specified Dynamixel
         * unit.
         */
        int getCWComplianceSlope();

        /**
         * The compliance feature can be utilized for absorbing shocks at the
         * output shaft.
         * @return the counter clockwise compliance slope of the specified
         * Dynamixel unit.
         */
        int getCCWComplianceSlope();

        /**
         * @return the requested goal angular position of the specified
         * Dynamixel unit.
         */
        int getGoalPosition();

        /**
         * This angular velocity is defined in range (0, 1023) i.e. (0, 0x3FF) in
         * hexadecimal notation. The maximum value (1023 or 0x3FF) corresponds to
         * 114 RPM (provided that there is enough power supplied).
         * Zero is a special value meaning that the largest possible velocity is
         * supplied for the configured voltage, e.g. no velocity control is
         * applied.
         * @return the angular velocity of the specified Dynamixel unit.
         */
        int getMovingSpeed();

        /**
         * @return the maximum torque output of the specified Dynamixel unit.
         */
        int getTorqueLimit();

        /**
         * @return the current angular position defined in range (0, 1023)
         * of the specified Dynamixel unit.
         */
        int getPresentPosition();

        /**
         * @return the current angular velocity of the specified Dynamixel unit.
         */
        int getPresentSpeed();

        /**
         * If the returned value is negative, the load is applied to the clockwise
         * direction.
         * If the returned value is positive, the load is applied to the counter
         * clockwise direction.
         * @return the magnitude of the load applied to the specified Dynamixel
         * unit.
         */
        int getPresentLoad();

        /**
         * @return the voltage currently applied to the specified Dynamixel
         * unit (in Volts).
         */
        int getPresentVoltage();

        /**
         * @return the internal temperature of the specified Dynamixel unit (in
         * Degrees Celsius).
         */
        int getPresentTemperature();

        /**
         * @return true if the specified Dynamixel unit is moving by its own
         * power; otherwise, false.
         */
        bool isMoving();

        /**
         * @return true if the specified Dynamixel unit is locked; otherwise,
         * false.
         */
        bool isLocked();

        /**
         * The initial value is set to 0x20 and its maximum value is 0x3FF.
         * @return the minimum current supplied to the motor of the specified
         * Dynamixel unit during operation.
         */
        int getPunch();

        /* High level mutators */

        /**
         * Set the *ID* for the specified Dynamixel unit.
         * i.e. the unique ID number assigned to actuators for identifying them.
         * Different IDs are required for each Dynamixel actuators that are on the
         * same network.
         * @param newId the new unique ID assigned to the selected
         * Dynamixel unit. It must be in range (0, 0xFE).
         */
        void setId(byte newId);

        /**
         * Set the *baud rate* for the specified Dynamixel unit.
         * i.e. set the connection speed with the actuator.
         * @param baudRate the new baud rate assigned to the selected
         * Dynamixel unit.
         */
        void setBaudRate(unsigned int baudRate);

        /**
         * Set the *return delay time* for the specified Dynamixel unit.
         * i.e. the time for the status packets to return after the instruction
         * packet is sent.
         * @param returnDelayTime the new return delay time. It must be in
         * range (0, 255).
         */
        void setReturnDelayTime(int returnDelayTime);

        /**
         * Set the *clockwise angle limit* of the specified Dynamixel unit to
         * the specified `angleLimit`.
         * The *goal position* should be higher or equal than this value, otherwise
         * the *Angle Limit Error Bit* (the second error bit of status packets)
         * will be set to `1`.
         * @param angleLimit the *clockwise angle limit* to be set for the
         * specified Dynamixel unit. It must be in range (0, 1023).
         */
        void setCWAngleLimit(short angleLimit);

        /**
         * Set he *counter clockwise angle limit* of the specified Dynamixel unit
         * to the specified `angleLimit`.
         * The *goal position* should be higher or equal than this value, otherwise
         * the *Angle Limit Error Bit* (the second error bit of status packets)
         * will be set to `1`.
         * @param angleLimit the *clockwise angle limit* to be set for the
         * specified Dynamixel unit. It must be in range (0, 1023).
         */
        void setCCWAngleLimit(short angleLimit);

        /**
         * Set the *moving speed* for the specified Dynamixel unit.
         * @param speed the new moving speed. It must be in range (0, 1023).
         */
        void setMovingSpeed(short speed);

        /**
         * Set the *goal position* for the specified Dynamixel unit.
         * @param speed the new goal position. It must be in range (0, 1023).
         */
        void setGoalPosition(short position);

    private:
        byte id;
        std::shared_ptr<SerialPort> port;

        std::function<void(bool)> callback;

        /**
         * Check error bit flags. And log them if there's any error.
         * @param errorCode error bit flags.
         */
        void checkError(byte errorCode);

        /**
         * Removes all irrelevant data from the status packet.
         * @param statusPacket a vector of bytes containing the status packet's data.
         * @return a vector of bytes containing the cleaned status packet's data.
         */
        std::vector<byte> cleanStatusPacket(std::vector<byte> &statusPacket);
    };
}