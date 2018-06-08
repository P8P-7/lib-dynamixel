#pragma once

namespace goliath::dynamixel {
    class Utils {
    public:
        using byte = unsigned char;

        static std::vector<byte> convertToHL(short pos) {
            return {(byte) pos, (byte) (pos >> 8)};
        }

        static short convertFromHL(byte hexL, byte hexH) {
            return (short) ((hexH << 8) + hexL);
        }

        static byte checkSum(std::vector<byte> data) {
            int cs = 0;

            // Skip first 2 items (padding)
            for (std::vector<int>::size_type i = 2; i != data.size(); i++) {
                cs += data[i];
            }

            return (byte) ~cs;
        }
    };
}
