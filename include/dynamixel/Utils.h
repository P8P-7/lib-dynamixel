#pragma once

#include <vector>

namespace goliath::dynamixel {
    class Utils {
    public:
        static std::vector<unsigned char> convertToHL(short pos);

        static short convertFromHL(unsigned char hexL, unsigned char hexH);

        static unsigned char checkSum(std::vector<unsigned char> data);
    };
}
