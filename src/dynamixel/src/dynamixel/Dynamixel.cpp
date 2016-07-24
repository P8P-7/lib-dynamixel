#include "stdio.h"
#include <string.h>

#include "dynamixel/Dynamixel.h"
#include "dynamixel/Utils.h"

Dynamixel::Dynamixel()
  : _recvWaitTimeMS(50)
{
}

Dynamixel::Dynamixel(byte id, SerialPort* port)
  : _id(id),
    _port(port)
{
}

void Dynamixel::Configure()
{
  Commands["Get"] = 2;
  Commands["Set"] = 3;
  ResponseLength["Get"] = 8;
  ResponseLength["Set"] = 6;
}

//
// MX28
//
void MX28::Configure()
{
  Addresses["Position"] = 36;
  Addresses["Speed"] = 32;
  Addresses["Goal"] = 30;
  Addresses["CCWComplianceSlope"] = 29;
  Addresses["CWComplianceSlope"] = 28;
  Addresses["CCWComplianceMargin"] = 27;
  Addresses["CWComplianceMargin"] = 26;
  Addresses["CCWAngleLimit"] = 8;
  Addresses["CWAngleLimit"] = 6;
  Dynamixel::Configure();
}

float MX28::posToAngle(short pos)
{
  float angle = 0;
  angle = (float)pos * 0.088f;
  return angle;
}

short MX28::angleToPos(float angle)
{
  short pos = 0;
  pos = (short)(angle/0.088f);
  return pos;
}

//
// AX12
//
void AX12::Configure()
{
  Addresses["Position"] = 36;
  Addresses["Speed"] = 32;
  Addresses["Goal"] = 30;
  Addresses["CCWComplianceSlope"] = 29;
  Addresses["CWComplianceSlope"] = 28;
  Addresses["CCWComplianceMargin"] = 27;
  Addresses["CWComplianceMargin"] = 26;
  Addresses["CCWAngleLimit"] = 8;
  Addresses["CWAngleLimit"] = 6;
  Dynamixel::Configure();
}

float AX12::posToAngle(short pos)
{
  float angle = 0;
  angle = (float)pos * 0.29f;
  return angle;
}

short AX12::angleToPos(float angle)
{
  short pos = 0;
  pos = (short)(angle/0.29f);
  return pos;
}

int Dynamixel::SendReceiveCommand(std::string command, std::string address,
				  std::vector<byte> data,
				  std::vector<byte>* outData)
{
  byte sendBuf[BufferSize] = {0};
  byte recvBuf[BufferSize] = {0};
  int retVal = 0;
  int responseLength = ResponseLength[command];
  int length = FormatCommand(Commands[command],
			     Addresses[address],
			     data,
			     sendBuf);
  // send
  long l = _port->sendArray(sendBuf, length);
  // sleep
  Utils::sleepMS(_recvWaitTimeMS);
  // recv 1
  int recvLen = _port->getArray(recvBuf, length); // receive once to get what we sent
  memset(recvBuf, 0, responseLength+1);
  // recv 2
  recvLen = _port->getArray(recvBuf, responseLength); // receive again to get the real data
  // check data
  if (recvLen >= responseLength) {
    int numValues = responseLength - 6;
    for (int i=0; i<numValues; i++) {
      outData->push_back(recvBuf[5 + i]);
    }
    return recvBuf[4]; // the error code if there is one
  }
  else {
    return -1;
  }
}

int Dynamixel::FormatCommand(byte command, byte address, std::vector<byte> values, byte* buffer)
{
  byte numberOfParameters = 0;

  //OXFF 0XFF ID LENGTH INSTRUCTION PARAMETER1 �PARAMETER N CHECK_SUM
  buffer[0] = 0xff;
  buffer[1] = 0xff;
  buffer[2] = _id;

  // bodyLength
  buffer[3] = 0; // temp

  //the instruction
  buffer[4] = command;

  // start of goal registers
  buffer[5] = address;

  //bytes to write
  for (int i=0; i<values.size(); i++) {
    buffer[6+i] = values[i];
  }
  numberOfParameters = values.size();

  // bodyLength
  buffer[3] = (byte)(numberOfParameters + 3);

  byte checksum = Utils::CheckSum(buffer, 6 + numberOfParameters);
  buffer[6 + numberOfParameters] = checksum;

  return 7 + numberOfParameters;
}

int Dynamixel::getPosition() 
{
  std::vector<byte> data = {2}; // number of bytes to read
  std::vector<byte> returnData;
  SendReceiveCommand("Get", "Position", data, &returnData);
  if (returnData.size() == 2) {
    return Utils::ConvertFromHL(returnData[0], returnData[1]);
  }
  return -1;
}

int Dynamixel::setPosition(int position) 
{
  byte posH, posL;
  Utils::ConvertToHL(position, &posH, &posL);
  std::vector<byte> data = {posH, posL};
  std::vector<byte> returnData;
  return SendReceiveCommand("Set", "Goal", data, &returnData);
}

int Dynamixel::setSpeed(int speed) 
{
  byte speedH, speedL;
  Utils::ConvertToHL(speed, &speedH, &speedL);
  std::vector<byte> data = {speedH, speedL};
  std::vector<byte> returnData;
  return SendReceiveCommand("Set", "Speed", data, &returnData);
}

int Dynamixel::setCWAngleLimit(int limit) 
{
  byte limitH, limitL;
  Utils::ConvertToHL(limit, &limitH, &limitL);
  std::vector<byte> data = {limitH, limitL};
  std::vector<byte> returnData;
  return SendReceiveCommand("Set", "CWAngleLimit", data, &returnData);
}

int Dynamixel::setCCWAngleLimit(int limit) 
{
  byte limitH, limitL;
  Utils::ConvertToHL(limit, &limitH, &limitL);
  std::vector<byte> data = {limitH, limitL};
  std::vector<byte> returnData;
  return SendReceiveCommand("Set", "CCWAngleLimit", data, &returnData);
}

int AX12::setCWComplianceMargin(int margin) 
{
  std::vector<byte> data = {margin};
  std::vector<byte> returnData;
  return SendReceiveCommand("Set", "CWComplianceMargin", data, &returnData);
}

int AX12::setCCWComplianceMargin(int margin) 
{
  std::vector<byte> data = {margin};
  std::vector<byte> returnData;
  return SendReceiveCommand("Set", "CCWComplianceMargin", data, &returnData);
}

int AX12::setCWComplianceSlope(int slope) 
{
  std::vector<byte> data = {slope};
  std::vector<byte> returnData;
  return SendReceiveCommand("Set", "CWComplianceSlope", data, &returnData);
}

int AX12::setCCWComplianceSlope(int slope) 
{
  std::vector<byte> data = {slope};
  std::vector<byte> returnData;
  return SendReceiveCommand("Set", "CCWComplianceSlope", data, &returnData);
}
