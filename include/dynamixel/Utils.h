#pragma once

#include <vector>

namespace goliath::dynamixel {
    class Utils {
    public:
        using byte = unsigned char;

        static std::vector<byte> convertToHL(short pos);

        static short convertFromHL(byte hexL, byte hexH);

        static byte checkSum(std::vector<byte> data);
    };
}
