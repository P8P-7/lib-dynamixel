#include "dynamixel/Dynamixel.h"

using namespace goliath::dynamixel;

Dynamixel::Dynamixel(byte id, SerialPort &port) : id(id),
                                                  port(port) {
}

void Dynamixel::setDirectionCallback(std::function<void(bool)> callback) {
    callback = std::move(callback);
}

size_t Dynamixel::sendCommand(Commands command, Addresses address, const std::vector<byte> &data) {
    std::vector<byte> buffer = getBuffer(command, address, data);

    if (callback) {
        callback(true);
    }

    // Write supplied data to the serial device.
    size_t bytesWritten = port.write(buffer);

    if (callback) {
        callback(false);
    }

    return bytesWritten;
}

std::vector<Dynamixel::byte>
Dynamixel::sendReceiveCommand(Commands command, Addresses address, const std::vector<byte> &data) {
    sendCommand(command, address, data);

    size_t responseLength = 6;
    if (command == Commands::Get) {
        responseLength += data[0];
    }

    // The structure of the status packet is as the following:
    // +----+----+--+------+-----+----------+---+-----------+---------+
    // |0XFF|0XFF|ID|LENGTH|ERROR|PARAMETER1|...|PARAMETER N|CHECK SUM|
    // +----+----+--+------+-----+----------+---+-----------+---------+
    std::vector<byte> recvBuffer = port.read(responseLength);

    // Assert the argument is a sequence with at least 6+ items.
    if (recvBuffer.size() < responseLength) {
        throw std::runtime_error(
                "Incomplete packet. Received " + std::to_string(recvBuffer.size()) + " instead of expected " +
                std::to_string(responseLength) + " bytes.");
    }

    // Check the header bytes.
    if (recvBuffer[0] != 0xff || recvBuffer[1] != 0xff) {
        throw std::runtime_error("Wrong header (should be '0xFF0xFF')");
    }

    // The error code if there is one.
    byte errorCode = recvBuffer[4];

    if (errorCode & (1u << 6u)) { // // Instruction error
        throw std::runtime_error("Instruction error: a undefined instruction is transmitted.");
    } else if (errorCode & (1u << 5u)) { // Overload error
        throw std::runtime_error(
                "Overload error: the current load cannot be controlled with the set maximum torque.");
    } else if (errorCode & (1u << 4u)) { // Checksum error
        throw std::runtime_error("Checksum error: the checksum of the transmitted instruction packet is invalid.");
    } else if (errorCode & (1u << 3u)) { // Range error
        throw std::runtime_error("Range error: given command is beyond the range of usage.");
    } else if (errorCode & (1u << 2u)) { // Overheating error
        throw std::runtime_error("Overheating error: internal temperature is out of the range.");
    } else if (errorCode & (1u << 1u)) { // Angle limit error
        throw std::runtime_error("Angle limit error: goal position is not between CW angle limit and CCW.");
    } else if (errorCode & (1u << 0u)) { // Input voltage error
        throw std::runtime_error("Input voltage error: applied voltage is out of the range.");
    }

    // Remove the checksum
    recvBuffer.pop_back();

    // Remove the first 5 elements, and shift everything else down by 5 indices.
    recvBuffer.erase(recvBuffer.begin(), recvBuffer.begin() + 5);

    return recvBuffer;
}

std::vector<Dynamixel::byte>
Dynamixel::getBuffer(Commands command, Addresses address, const std::vector<byte> &values) {
    std::vector<byte> buffer;

    size_t numberOfParameters = values.size();

    // The structure of the instruction packet is as the following:
    // +----+----+--+------+-----------+----------+---+-----------+---------+
    // |0XFF|0XFF|ID|LENGTH|INSTRUCTION|PARAMETER1|...|PARAMETER N|CHECK SUM|
    // +----+----+--+------+-----------+----------+---+-----------+---------+
    buffer.push_back(0xff);
    buffer.push_back(0xff);
    buffer.push_back(id);

    // bodyLength
    buffer.push_back(static_cast<byte>(numberOfParameters + 3));

    // the instruction
    buffer.push_back(static_cast<byte>(command));

    // start of goal registers
    buffer.push_back(static_cast<byte>(address));

    // bytes to write
    buffer.insert(buffer.end(), values.begin(), values.end());

    // checksum
    buffer.push_back(Utils::checkSum(buffer));

    return buffer;
}

int Dynamixel::getPosition() {
    std::vector<byte> data = {2}; // number of bytes to read
    std::vector<byte> returnData = sendReceiveCommand(Commands::Get, Addresses::Position, data);

    if (returnData.size() == 2) {
        return Utils::convertFromHL(returnData[0], returnData[1]);
    }

    return -1;
}

int Dynamixel::getCurrentLoad() {
    std::vector<byte> data = {2}; // number of bytes to read
    std::vector<byte> returnData = sendReceiveCommand(Commands::Get, Addresses::Load, data);

    if (returnData.size() == 2) {
        return Utils::convertFromHL(returnData[0], returnData[1]);
    }

    return -1;
}

void Dynamixel::setGoalPosition(short goal) {
    std::vector<byte> data = Utils::convertToHL(goal);
    sendReceiveCommand(Commands::Set, Addresses::Goal, data);
}

void Dynamixel::setMovingSpeed(short speed) {
    std::vector<byte> data = Utils::convertToHL(speed);
    sendReceiveCommand(Commands::Set, Addresses::MovingSpeed, data);
}

int Dynamixel::getCWAngleLimit() {
    std::vector<byte> data = {2}; // number of bytes to read
    std::vector<byte> returnData = sendReceiveCommand(Commands::Get, Addresses::CWAngleLimit, data);

    if (returnData.size() == 2) {
        return Utils::convertFromHL(returnData[0], returnData[1]);
    }

    return -1;
}

void Dynamixel::setCWAngleLimit(short limit) {
    std::vector<byte> data = Utils::convertToHL(limit);
    sendReceiveCommand(Commands::Set, Addresses::CWAngleLimit, data);
}

int Dynamixel::getCCWAngleLimit() {
    std::vector<byte> data = {2}; // number of bytes to read
    std::vector<byte> returnData = sendReceiveCommand(Commands::Get, Addresses::CCWAngleLimit, data);

    if (returnData.size() == 2) {
        return Utils::convertFromHL(returnData[0], returnData[1]);
    }

    return -1;
}

void Dynamixel::setCCWAngleLimit(short limit) {
    std::vector<byte> data = Utils::convertToHL(limit);
    sendReceiveCommand(Commands::Set, Addresses::CCWAngleLimit, data);
}
