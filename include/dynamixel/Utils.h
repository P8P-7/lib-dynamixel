#pragma once

#include <cstring>

class Utils {
public:
    static std::string printBuffer(byte data[], int length) {
        char bytes[length * 3 + 1];
        memset(bytes, 0, length * 3 + 1);
        for (int i = 0; i < length; i++) {
            sprintf(bytes, "%s%02X ", bytes, data[i]);
        }
        std::string str = bytes;
        return str;
    }

    static void convertToHL(short pos, byte *hexH, byte *hexL) {
        *hexH = (byte) (pos >> 8);
        *hexL = (byte) pos;
    }

    static short convertFromHL(byte hexL, byte hexH) {
        return (short) ((hexH << 8) + hexL);
    }

    static byte checkSum(byte data[], int length) {
        int cs = 0;
        for (int i = 2; i < length; i++) {
            cs += data[i];
        }
        return (byte) ~cs;
    }
};
