#pragma once

class Utils {
public:
    static std::string printBuffer(byte data[], int length) {
        char bytes[length * 3 + 1] = {0};
        for (int i = 0; i < length; i++) {
            snprintf(bytes, sizeof(bytes) - 3 * (i), "%02X ", data[i]);
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
