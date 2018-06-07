#include "dynamixel/Dynamixel.h"

Dynamixel::Dynamixel(byte id, SerialPort &port) : _id(id),
                                                  _port(port) {
}

void Dynamixel::setDirectionCallback(std::function<void(bool)> callback) {
    _callback = std::move(callback);
}

size_t Dynamixel::sendCommand(Commands command, Addresses address, const std::vector<byte> &data) {
    std::vector<byte> buffer = getBuffer(command, address, data);

    if (_callback) {
        _callback(true);
    }

    // Write supplied data to the serial device.
    size_t bytesWritten = _port.write(buffer);

    if (_callback) {
        _callback(false);
    }

    return bytesWritten;
}

std::vector<byte> Dynamixel::sendReceiveCommand(Commands command, Addresses address, const std::vector<byte> &data) {
    sendCommand(command, address, data);

    size_t responseLength = 6;
    if (command == Commands::Get) {
        responseLength += data[0];
    }

    // recv 1 (omitted)

    // recv 2
    std::vector<byte> recvBuffer = _port.read(responseLength); // receive again to get the real data

    // check data
    if (recvBuffer.size() >= responseLength) {
        // the error code if there is one
        int errorCode = recvBuffer[4];

        switch (errorCode) {
            case 1: // Input voltage error
                throw std::runtime_error("Input voltage error: applied voltage is out of the range.");
            case 2: // Angle limit error
                throw std::runtime_error("Angle limit error: goal position is not between CW angle limit and CCW.");
            case 4: // Overheating error
                throw std::runtime_error("Overheating error: internal temperature is out of the range.");
            case 8: // Range error
                throw std::runtime_error("Range error: given command is beyond the range of usage.");
            case 16: // Checksum error
                throw std::runtime_error(
                        "Checksum error: the checksum of the transmitted instruction packet is invalid.");
            case 32: // Overload
                throw std::runtime_error(
                        "Overload error: the current load cannot be controlled with the set maximum torque.");
            case 64: // Instruction error
                throw std::runtime_error("Instruction error: a undefined instruction is transmitted.");
            default:
                break;
        }

        // Remove the first 5 elements, and shift everything else down by 5 indices
        recvBuffer.erase(recvBuffer.begin(), recvBuffer.begin() + 5);

        return recvBuffer;
    } else {
        throw std::runtime_error(
                "Error reading data. Received " + std::to_string(recvBuffer.size()) + " instead of expected " +
                std::to_string(responseLength) + " bytes.");
    }
}

std::vector<byte> Dynamixel::getBuffer(Commands command, Addresses address, const std::vector<byte> &values) {
    std::vector<byte> buffer;

    size_t numberOfParameters = values.size();

    // OXFF 0XFF ID LENGTH INSTRUCTION PARAMETER1 PARAMETER N CHECK_SUM
    buffer.push_back(0xff);
    buffer.push_back(0xff);
    buffer.push_back(_id);

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
    byte posH, posL;
    Utils::convertToHL(goal, &posH, &posL);
    std::vector<byte> data = {posL, posH};
    sendReceiveCommand(Commands::Set, Addresses::Goal, data);
}

void Dynamixel::setMovingSpeed(short speed) {
    byte speedH, speedL;
    Utils::convertToHL(speed, &speedH, &speedL);
    std::vector<byte> data = {speedL, speedH};
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
    byte limitH, limitL;
    Utils::convertToHL(limit, &limitH, &limitL);
    std::vector<byte> data = {limitL, limitH};
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
    byte limitH, limitL;
    Utils::convertToHL(limit, &limitH, &limitL);
    std::vector<byte> data = {limitL, limitH};
    sendReceiveCommand(Commands::Set, Addresses::CCWAngleLimit, data);
}
