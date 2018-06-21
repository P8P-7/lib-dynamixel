#include "dynamixel/Utils.h"

using namespace goliath::dynamixel;

std::vector<unsigned char> Utils::convertToHL(short pos) {
    return {(unsigned char) pos, (unsigned char) (pos >> 8)};
}

short Utils::convertFromHL(unsigned char hexL, unsigned char hexH) {
    return (short) ((hexH << 8) + hexL);
}

unsigned char Utils::checkSum(std::vector<unsigned char> data) {
    int cs = 0;

    // Skip first 2 items (padding)
    for (std::size_t i = 2; i < data.size(); i++) {
        cs += data[i];
    }

    return (unsigned char) ~cs;
}
