/*
 * Utils.h
 *
 *  Created on: Mar 3, 2012
 *      Author: jose
 */

#ifndef UTILS_H_
#define UTILS_H_

#include <string>
#include <unistd.h>

class Utils {
 public:
  static std::string PrintBuffer(byte data[], int length) {
    char *bytes = new char[length*3 + 1];
    for (int i=0; i<length; i++) {
      sprintf(bytes,"%s%02X ", bytes, data[i]);
    }
    std::string str = bytes;
    delete bytes;
    return str;
  }

  static void ConvertToHL(short pos, byte *hexH, byte *hexL) {
    *hexH = (byte)(pos >> 8);
    *hexL = (byte)pos;
  }

  static short ConvertFromHL(byte hexH, byte hexL) {
    return (short)((hexL << 8) + hexH);
  }

  static byte CheckSum(byte  data[], int length) {
    int cs = 0;
    for (int i = 2; i < length; i++)
      {
	cs += data[i];
      }            
    return (byte)~cs;
  }

  static void sleepMS(int ms) {
    sleepMS((long)ms);
  }

  static void sleepMS(long ms) {
    usleep(ms*1000);
  }
};

#endif /* UTILS_H_ */
