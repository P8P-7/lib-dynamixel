#ifndef DYNAMIXEL_H_
#define DYNAMIXEL_H_

typedef unsigned char byte;

#include <map>
#include <vector>
#include <string>
#include "SerialPort.h"

static const int BufferSize=1024;
byte buffer[BufferSize];
byte bufferIn[BufferSize];

class Dynamixel {

 public:

  std::map<std::string, byte> Addresses;
  std::map<std::string, byte> Commands;
  std::map<std::string, int> CommandResponseLengths;

  static float posToAngle(short pos) = 0;
  static short angleToPos(float angle) = 0;

  int getPosition();
  int setPosition(int position);
  
  int setSpeed(int speed);

  int setCCWAngleLimit(int limit);
  int setCWAngleLimit(int limit);

  int setCCWComplianceMargin(int margin);
  int setCWComplianceMargin(int margin);

  int setCCWComplianceSlope(int slope);
  int setCWComplianceSlope(int slope);
		
  int sendTossModeCommand(SerialPort *serialPort);

 private:

  byte _id;
  SerialPort* _port;
  int _recvWaitTimeMS; //=50

  void toHexHLConversion(short pos, byte *hexH, byte *hexL);
  short fromHexHLConversion(byte hexH, byte hexL);
  byte checkSum(byte  data[], int length);

  void WriteHeader(byte* buffer, byte length = 0);
  int FormatCommand(byte command, byte address, std::vector<byte>, byte* buffer);
  int FormatCommand(byte command, byte address, byte* buffer);
  int SendReceive(byte* buffer, int length, int responseLength);

  void Configure() = 0;

 public:
  Dynamixel();
  Dynamixel(byte id, SerialPort* port);
};

class AX12 : Dynamixel {
 private:
  void Configure();
 public:
  static float posToAngle(short pos);
  static short angleToPos(float angle);
};

class MX28 : Dynamixel {
 private:
  void Configure();
 public:
  static float posToAngle(short pos);
  static short angleToPos(float angle);

  int setPGain(int pGain);
  int setIGain(int pGain);
  int setDGain(int pGain);
};

#endif
