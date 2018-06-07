#pragma once

class Utils {
public:
    static void convertToHL(short pos, byte *hexH, byte *hexL) {
        *hexH = (byte) (pos >> 8);
        *hexL = (byte) pos;
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
