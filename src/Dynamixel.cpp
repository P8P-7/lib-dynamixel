#include "dynamixel/Dynamixel.h"

using namespace goliath::dynamixel;

Dynamixel::Dynamixel(byte id, SerialPort &port) : id(id),
                                                  port(port) {
}

void Dynamixel::setDirectionCallback(std::function<void(bool)> callback) {
    callback = std::move(callback);
}

std::vector<Dynamixel::byte> Dynamixel::send(Instruction instruction, const std::vector<byte> &data) {
    // The structure of the instruction packet is as the following:
    // +----+----+--+------+-----------+----------+---+-----------+---------+
    // |0XFF|0XFF|ID|LENGTH|INSTRUCTION|PARAMETER1|...|PARAMETER N|CHECK SUM|
    // +----+----+--+------+-----------+----------+---+-----------+---------+
    std::vector<byte> instructionPacket = getInstructionPacket(instruction, data);

    if (callback) {
        callback(true);
    }

    size_t bytesWritten = port.write(instructionPacket);

    if (callback) {
        callback(false);
    }

    size_t responseLength = 6;
    if (instruction == Instruction::Read) {
        responseLength += data[0];
    }

    // The structure of the status packet is as the following:
    // +----+----+--+------+-----+----------+---+-----------+---------+
    // |0XFF|0XFF|ID|LENGTH|ERROR|PARAMETER1|...|PARAMETER N|CHECK SUM|
    // +----+----+--+------+-----+----------+---+-----------+---------+
    std::vector<byte> statusPacket = port.read(responseLength);

    // Assert the argument is a sequence with at least 6+ items.
    if (statusPacket.size() < responseLength) {
        throw std::runtime_error(
                "Incomplete packet. Received " + std::to_string(statusPacket.size()) + " instead of expected " +
                std::to_string(responseLength) + " bytes.");
    }

    // Check the header bytes.
    if (statusPacket[0] != 0xff || statusPacket[1] != 0xff) {
        throw std::runtime_error("Wrong header (should be '0xFF0xFF')");
    }

    // Check the error code, if there's one.
    checkError(statusPacket[4]);

    return statusPacket;
}

std::vector<Dynamixel::byte> Dynamixel::getInstructionPacket(Instruction instruction, const std::vector<byte> &data) {
    std::vector<byte> instructionPacket;

    size_t numberOfParameters = data.size();

    instructionPacket.push_back(0xff);
    instructionPacket.push_back(0xff);
    instructionPacket.push_back(id);

    // bodyLength
    instructionPacket.push_back(static_cast<byte>(numberOfParameters + 2));

    // the instruction
    instructionPacket.push_back(static_cast<byte>(instruction));

    // bytes to write
    if (numberOfParameters > 0) {
        instructionPacket.insert(instructionPacket.end(), data.begin(), data.end());
    }

    // checksum
    instructionPacket.push_back(Utils::checkSum(instructionPacket));

    return instructionPacket;
}

void Dynamixel::checkError(byte errorCode) {
    if (errorCode & (1u << 6u)) { // Instruction error
        BOOST_LOG_TRIVIAL(warning) << "Instruction error: a undefined instruction is transmitted.";
    }
    if (errorCode & (1u << 5u)) { // Overload error
        BOOST_LOG_TRIVIAL(warning)
            << "Overload error: the current load cannot be controlled with the set maximum torque.";
    }
    if (errorCode & (1u << 4u)) { // Checksum error
        BOOST_LOG_TRIVIAL(warning) << "Checksum error: the checksum of the transmitted instruction packet is invalid.";
    }
    if (errorCode & (1u << 3u)) { // Range error
        BOOST_LOG_TRIVIAL(warning) << "Range error: given command is beyond the range of usage.";
    }
    if (errorCode & (1u << 2u)) { // Overheating error
        BOOST_LOG_TRIVIAL(warning) << "Overheating error: internal temperature is out of the range.";
    }
    if (errorCode & (1u << 1u)) { // Angle limit error
        BOOST_LOG_TRIVIAL(warning) << "Angle limit error: goal position is not between CW angle limit and CCW.";
    }
    if (errorCode & (1u << 0u)) { // Input voltage error
        BOOST_LOG_TRIVIAL(warning) << "Input voltage error: applied voltage is out of the range.";
    }
}

std::vector<Dynamixel::byte> Dynamixel::cleanStatusPacket(std::vector<byte> &statusPacket) {
    // Remove the checksum
    statusPacket.pop_back();

    // Remove the first 5 elements, and shift everything else down by 5 indices.
    statusPacket.erase(statusPacket.begin(), statusPacket.begin() + 5);

    return statusPacket;
}

std::vector<Dynamixel::byte> Dynamixel::readData(Address address, size_t length) {
    std::vector<byte> params = {static_cast<byte>(address), static_cast<byte>(length)};
    std::vector<byte> statusPacket = send(Instruction::Read, params);

    return cleanStatusPacket(statusPacket);
}

void Dynamixel::writeData(Address address, const std::vector<byte> &data) {
    std::vector<byte> params = {static_cast<byte>(address)};
    params.insert(params.end(), data.begin(), data.end());

    send(Instruction::Write, data);
}

bool Dynamixel::ping() {
    std::vector<byte> statusPacket = send(Instruction::Ping, {});

    if (statusPacket[2] == id) {
        return true;
    }

    return false;
}

int Dynamixel::getFirmwareVersion() {
    std::vector<byte> data = readData(Address::FirmwareVersion, 1);

    return data[0];
}

unsigned int Dynamixel::getBaudRate() {
    std::vector<byte> data = readData(Address::BaudRate, 1);
    byte rawValue = data[0];

    return static_cast<unsigned int>(2000000 / (rawValue + 1));
}

int Dynamixel::getReturnDelayTime() {
    std::vector<byte> data = readData(Address::ReturnDelayTime, 1);
    byte rawValue = data[0];

    return 2 * rawValue;
}

int Dynamixel::getCWAngleLimit() {
    std::vector<byte> data = readData(Address::CWAngleLimit, 2);

    if (data.size() == 2) {
        return Utils::convertFromHL(data[0], data[1]);
    }

    return -1;
}

int Dynamixel::getCCWAngleLimit() {
    std::vector<byte> data = readData(Address::CCWAngleLimit, 2);

    if (data.size() == 2) {
        return Utils::convertFromHL(data[0], data[1]);
    }

    return -1;
}

int Dynamixel::getMaxTemperature() {
    std::vector<byte> data = readData(Address::HighestLimitTemperature, 1);

    return data[0];
}

int Dynamixel::getMinVoltage() {
    std::vector<byte> data = readData(Address::LowestLimitVoltage, 1);
    byte rawValue = data[0];

    return rawValue / 10;
}

int Dynamixel::getMaxVoltage() {
    std::vector<byte> data = readData(Address::HighestLimitVoltage, 1);
    byte rawValue = data[0];

    return rawValue / 10;
}

int Dynamixel::getMaxTorque() {
    std::vector<byte> data = readData(Address::MaxTorque, 2);

    if (data.size() == 2) {
        return Utils::convertFromHL(data[0], data[1]);
    }

    return -1;
}

int Dynamixel::getStatusReturnLevel() {
    std::vector<byte> data = readData(Address::StatusReturnLevel, 1);

    return data[0];
}

int Dynamixel::getDownCalibration() {
    std::vector<byte> data = readData(Address::DownCalibration, 2);

    if (data.size() == 2) {
        return Utils::convertFromHL(data[0], data[1]);
    }

    return -1;
}

int Dynamixel::getUpCalibration() {
    std::vector<byte> data = readData(Address::UpCalibration, 2);

    if (data.size() == 2) {
        return Utils::convertFromHL(data[0], data[1]);
    }

    return -1;
}

bool Dynamixel::isTorqueEnabled() {
    std::vector<byte> data = readData(Address::TorqueStatus, 1);

    return data[0] == 1;
}

bool Dynamixel::isLedEnabled() {
    std::vector<byte> data = readData(Address::LEDStatus, 1);

    return data[0] == 1;
}

int Dynamixel::getCWComplianceMargin() {
    std::vector<byte> data = readData(Address::CWComplianceMargin, 1);

    return data[0];
}

int Dynamixel::getCCWComplianceMargin() {
    std::vector<byte> data = readData(Address::CCWComplianceMargin, 1);

    return data[0];
}

int Dynamixel::getCWComplianceSlope() {
    std::vector<byte> data = readData(Address::CWComplianceSlope, 1);

    return data[0];
}

int Dynamixel::getCCWComplianceSlope() {
    std::vector<byte> data = readData(Address::CCWComplianceSlope, 1);

    return data[0];
}

int Dynamixel::getGoalPosition() {
    std::vector<byte> data = readData(Address::GoalPosition, 2);

    if (data.size() == 2) {
        return Utils::convertFromHL(data[0], data[1]);
    }

    return -1;
}

int Dynamixel::getMovingSpeed() {
    std::vector<byte> data = readData(Address::MovingSpeed, 2);

    if (data.size() == 2) {
        return Utils::convertFromHL(data[0], data[1]);
    }

    return -1;
}

int Dynamixel::getTorqueLimit() {
    std::vector<byte> data = readData(Address::TorqueLimit, 2);

    if (data.size() == 2) {
        return Utils::convertFromHL(data[0], data[1]);
    }

    return -1;
}

int Dynamixel::getPresentPosition() {
    std::vector<byte> data = readData(Address::PresentPosition, 2);

    if (data.size() == 2) {
        return Utils::convertFromHL(data[0], data[1]);
    }

    return -1;
}

int Dynamixel::getPresentSpeed() {
    std::vector<byte> data = readData(Address::PresentSpeed, 2);

    if (data.size() == 2) {
        return Utils::convertFromHL(data[0], data[1]);
    }

    return -1;
}

int Dynamixel::getPresentLoad() {
    std::vector<byte> data = readData(Address::PresentLoad, 2);

    if (data.size() == 2) {
        int loadDirection = (data[1] & (1u << 2u)) == 0u ? -1 : 1;

        data[1] = static_cast<unsigned char>(3u & data[1]);

        int absLoad = Utils::convertFromHL(data[0], data[1]);

        return loadDirection * absLoad;
    }

    return -1;
}

int Dynamixel::getPresentVoltage() {
    std::vector<byte> data = readData(Address::PresentVoltage, 1);
    byte rawValue = data[0];

    return rawValue / 10;
}

int Dynamixel::getPresentTemperature() {
    std::vector<byte> data = readData(Address::PresentTemperature, 1);

    return data[0];
}

bool Dynamixel::isMoving() {
    std::vector<byte> data = readData(Address::Moving, 1);

    return data[0] == 1;
}

bool Dynamixel::isLocked() {
    std::vector<byte> data = readData(Address::Lock, 1);

    return data[0] == 1;
}

int Dynamixel::getPunch() {
    std::vector<byte> data = readData(Address::Punch, 2);

    if (data.size() == 2) {
        return Utils::convertFromHL(data[0], data[1]);
    }

    return -1;
}

void Dynamixel::setId(byte newId) {
    writeData(Address::ID, {id});
    id = newId;
}

void Dynamixel::setBaudRate(unsigned int baudRate) {
    writeData(Address::BaudRate, {static_cast<byte>(round(2000000 / baudRate) - 1)});
}

void Dynamixel::setReturnDelayTime(int returnDelayTime) {
    writeData(Address::ReturnDelayTime, {static_cast<byte>(returnDelayTime)});
}

void Dynamixel::setCWAngleLimit(short angleLimit) {
    std::vector<byte> data = Utils::convertToHL(angleLimit);
    writeData(Address::CWAngleLimit, data);
}

void Dynamixel::setCCWAngleLimit(short angleLimit) {
    std::vector<byte> data = Utils::convertToHL(angleLimit);
    writeData(Address::CCWAngleLimit, data);
}

void Dynamixel::setMovingSpeed(short speed) {
    std::vector<byte> data = Utils::convertToHL(speed);
    writeData(Address::MovingSpeed, data);
}

void Dynamixel::setGoalPosition(short position) {
    std::vector<byte> data = Utils::convertToHL(position);
    writeData(Address::GoalPosition, data);
}
