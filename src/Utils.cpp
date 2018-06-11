#include "dynamixel/Utils.h"

using namespace goliath::dynamixel;

std::vector<Utils::byte> Utils::convertToHL(short pos) {
    return {(byte) pos, (byte)(pos >> 8)};
}

short Utils::convertFromHL(byte hexL, byte hexH) {
    return (short) ((hexH << 8) + hexL);
}

Utils::byte Utils::checkSum(std::vector<byte> data) {
    int cs = 0;

    // Skip first 2 items (padding)
    for (std::vector<int>::size_type i = 2; i != data.size(); i++) {
        cs += data[i];
    }

    return (byte) ~cs;
}
