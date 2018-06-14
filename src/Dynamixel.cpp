#include "dynamixel/Dynamixel.h"

using namespace goliath::dynamixel;

Dynamixel::Dynamixel(byte id, std::shared_ptr<SerialPort> port) : id(id),
                                                                  port(std::move(port)) {
}

void Dynamixel::setDirectionCallback(std::function<void(bool)> callback) {
    this->callback = std::move(callback);
}

std::vector<Dynamixel::byte> Dynamixel::send(Instruction instruction, const std::vector<byte> &data) {
    // The structure of the instruction packet is as the following:
    // +----+----+--+------+-----------+----------+---+-----------+---------+
    // |0xFF|0xFF|ID|LENGTH|INSTRUCTION|PARAMETER1|...|PARAMETER N|CHECK SUM|
    // +----+----+--+------+-----------+----------+---+-----------+---------+
    std::vector<byte> instructionPacket = getInstructionPacket(instruction, data);

    if (callback) {
        callback(true);
    }

    port->write(instructionPacket);

    if (callback) {
        callback(false);
    }

    size_t responseLength = 6;
    if (instruction == Instruction::Read) {
        responseLength += data[1];
    }

    // The structure of the status packet is as the following:
    // +----+----+--+------+-----+----------+---+-----------+---------+
    // |0xFF|0xFF|ID|LENGTH|ERROR|PARAMETER1|...|PARAMETER N|CHECK SUM|
    // +----+----+--+------+-----+----------+---+-----------+---------+
    std::vector<byte> statusPacket = port->read(responseLength);

    // Assert the packet is a sequence with at least 6+ items.
    if (statusPacket.size() < responseLength) {
        throw std::runtime_error(
                "Incomplete packet. Received " + std::to_string(statusPacket.size()) + " instead of expected " +
                std::to_string(responseLength) + " bytes.");
    }

    // Check the header bytes.
    if (statusPacket[0] != '\xFF' || statusPacket[1] != '\xFF') {
        throw std::runtime_error("Wrong header; should be equal to [0xFF, 0xFF]");
    }

    // Check the error code, if there's one.
    checkError(statusPacket[4]);

    return statusPacket;
}

std::vector<Dynamixel::byte> Dynamixel::getInstructionPacket(Instruction instruction, const std::vector<byte> &data) {
    std::vector<byte> instructionPacket;

    size_t numberOfParameters = data.size();

    instructionPacket.push_back('\xFF');
    instructionPacket.push_back('\xFF');
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
    byte checkSum = Utils::checkSum(instructionPacket);
    instructionPacket.push_back(checkSum);

    return instructionPacket;
}

void Dynamixel::checkError(byte errorCode) {
    if ((errorCode & static_cast<byte>(Error::Instruction)) != 0u) { // Instruction error
        BOOST_LOG_TRIVIAL(warning) << "Instruction error: a undefined instruction is transmitted.";
    }
    if ((errorCode & static_cast<byte>(Error::Overload)) != 0u) { // Overload error
        BOOST_LOG_TRIVIAL(warning)
            << "Overload error: the current load cannot be controlled with the set maximum torque.";
    }
    if ((errorCode & static_cast<byte>(Error::Checksum)) != 0u) { // Checksum error
        BOOST_LOG_TRIVIAL(warning) << "Checksum error: the checksum of the transmitted instruction packet is invalid.";
    }
    if ((errorCode & static_cast<byte>(Error::Range)) != 0u) { // Range error
        BOOST_LOG_TRIVIAL(warning) << "Range error: given command is beyond the range of usage.";
    }
    if ((errorCode & static_cast<byte>(Error::Overheat)) != 0u) { // Overheating error
        BOOST_LOG_TRIVIAL(warning) << "Overheating error: internal temperature is out of the range.";
    }
    if ((errorCode & static_cast<byte>(Error::AngleLimit)) != 0u) { // Angle limit error
        BOOST_LOG_TRIVIAL(warning) << "Angle limit error: goal position is not between CW angle limit and CCW.";
    }
    if ((errorCode & static_cast<byte>(Error::InputVoltage)) != 0u) { // Input voltage error
        BOOST_LOG_TRIVIAL(warning) << "Input voltage error: applied voltage is out of the range.";
    }
}

int Dynamixel::readData(Address address, size_t length) {
    std::vector<byte> params = {static_cast<byte>(address), static_cast<byte>(length)};
    std::vector<byte> statusPacket = send(Instruction::Read, params);
    std::vector<byte> data = cleanStatusPacket(statusPacket);

    // Assert that the packet is the correct size.
    if (data.size() != length) {
        return -1;
    }

    if (data.size() == 2) {
        return Utils::convertFromHL(data[0], data[1]);
    }

    return data[0];
}

void Dynamixel::writeData(Address address, const std::vector<byte> &data) {
    std::vector<byte> params = {static_cast<byte>(address)};
    params.insert(params.end(), data.begin(), data.end());

    send(Instruction::Write, params);
}

std::vector<Dynamixel::byte> Dynamixel::cleanStatusPacket(std::vector<byte> &statusPacket) {
    // Remove the checksum.
    statusPacket.pop_back();

    // Remove the first 5 elements, and shift everything else down by 5 indices.
    statusPacket.erase(statusPacket.begin(), statusPacket.begin() + 5);

    return statusPacket;
}

bool Dynamixel::ping() {
    std::vector<byte> statusPacket = send(Instruction::Ping, {});

    return statusPacket[2] == id;
}

int Dynamixel::getFirmwareVersion() {
    return readData(Address::FirmwareVersion, 1);
}

unsigned int Dynamixel::getBaudRate() {
    int value = readData(Address::BaudRate, 1);

    if (value == -1) {
        return 0;
    }

    return static_cast<unsigned int>(2000000 / (value + 1));
}

int Dynamixel::getReturnDelayTime() {
    int value = readData(Address::ReturnDelayTime, 1);

    if (value == -1) {
        return value;
    }

    return 2 * value;
}

int Dynamixel::getCWAngleLimit() {
    return readData(Address::CWAngleLimit, 2);
}

int Dynamixel::getCCWAngleLimit() {
    return readData(Address::CCWAngleLimit, 2);
}

int Dynamixel::getMaxTemperature() {
    return readData(Address::HighestLimitTemperature, 1);
}

int Dynamixel::getMinVoltage() {
    int value = readData(Address::LowestLimitVoltage, 1);

    if (value == -1) {
        return value;
    }

    return value / 10;
}

int Dynamixel::getMaxVoltage() {
    int value = readData(Address::HighestLimitVoltage, 1);

    if (value == -1) {
        return value;
    }

    return value / 10;
}

int Dynamixel::getMaxTorque() {
    return readData(Address::MaxTorque, 2);
}

int Dynamixel::getStatusReturnLevel() {
    return readData(Address::StatusReturnLevel, 1);
}

int Dynamixel::getDownCalibration() {
    return readData(Address::DownCalibration, 2);
}

int Dynamixel::getUpCalibration() {
    return readData(Address::UpCalibration, 2);
}

bool Dynamixel::isTorqueEnabled() {
    return readData(Address::TorqueStatus, 1) == 1;
}

bool Dynamixel::isLedEnabled() {
    return readData(Address::LEDStatus, 1) == 1;
}

int Dynamixel::getCWComplianceMargin() {
    return readData(Address::CWComplianceMargin, 1);
}

int Dynamixel::getCCWComplianceMargin() {
    return readData(Address::CCWComplianceMargin, 1);
}

int Dynamixel::getCWComplianceSlope() {
    return readData(Address::CWComplianceSlope, 1);
}

int Dynamixel::getCCWComplianceSlope() {
    return readData(Address::CCWComplianceSlope, 1);
}

int Dynamixel::getGoalPosition() {
    return readData(Address::GoalPosition, 2);
}

int Dynamixel::getMovingSpeed() {
    return readData(Address::MovingSpeed, 2);
}

int Dynamixel::getTorqueLimit() {
    return readData(Address::TorqueLimit, 2);
}

int Dynamixel::getPresentPosition() {
    return readData(Address::PresentPosition, 2);
}

int Dynamixel::getPresentSpeed() {
    return readData(Address::PresentSpeed, 2);
}

int Dynamixel::getPresentLoad() {
    std::vector<byte> params = {static_cast<byte>(Address::PresentLoad), 2};
    std::vector<byte> statusPacket = send(Instruction::Read, params);
    std::vector<byte> data = cleanStatusPacket(statusPacket);

    // Assert that the packet is the correct size.
    if (data.size() == 2) {
        int loadDirection = (data[1] & (1u << 2u)) == 0u ? -1 : 1;

        data[1] = static_cast<unsigned char>(3u & data[1]);

        int absLoad = Utils::convertFromHL(data[0], data[1]);

        return loadDirection * absLoad;
    }

    return -1;
}

int Dynamixel::getPresentVoltage() {
    int value = readData(Address::PresentVoltage, 1);

    if (value == -1) {
        return value;
    }

    return value / 10;
}

int Dynamixel::getPresentTemperature() {
    int value = readData(Address::PresentTemperature, 1);

    if (value == -1) {
        return value;
    }

    return value / 10;
}

bool Dynamixel::isMoving() {
    return readData(Address::Moving, 1) == 1;
}

bool Dynamixel::isLocked() {
    return readData(Address::Lock, 1) == 1;
}

int Dynamixel::getPunch() {
    return readData(Address::Punch, 2);
}

int Dynamixel::getId() const {
    return id;
}

void Dynamixel::setId(byte newId) {
    writeData(Address::ID, {id});
    id = newId;
}

void Dynamixel::setBaudRate(unsigned int baudRate) {
    writeData(Address::BaudRate, {static_cast<byte>(round(2000000.0 / baudRate) - 1)});
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
