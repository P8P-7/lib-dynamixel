#include "stdio.h"
#include <string.h>

#include "dynamixel/Dynamixel.h"
#include "dynamixel/Utils.h"

Dynamixel::Dynamixel()
  : _recvWaitTimeMS(50)
{
  Configure();
}

Dynamixel::Configure()
{
  Commands["Get"] = 2;
  Commands["Set"] = 3;
  CommandResponseLength["Get"] = 8;
  CommandResponseLength["Set"] = 6;
}

Dynamixel::Dynamixel(byte id, SerialPort* port)
  : _id(id),
    _port(port),
    Dynamixel()
{
}

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

loat MX28::posToAngle(short pos)
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

void Dynamixel::toHexHLConversion(short pos, byte *hexH, byte *hexL)
{    
  *hexH = (byte)(pos >> 8);
  *hexL = (byte)pos;
}

short Dynamixel::fromHexHLConversion(byte hexH, byte hexL)
{
  return (short)((hexL << 8) + hexH);
}

byte Dynamixel::checkSum(byte  data[], int length)
{
  int cs = 0;
  for (int i = 2; i < length; i++)
    {
      cs += data[i];
    }            
  return (byte)~cs;
}

int Dynamixel::SendReceive(byte* buffer, int length, int responseLength)
{
  byte recvBuf[_bufferSize] = {0};
  long l = port->sendArray(buffer, length);
  Utils::sleepMS(recvWaitTimeMS);
  int n = port->getArray(recvBuf, length); // receive once to get what we sent
  memset(recvBuf, 0, responseLength);
  n = port->getArray(recvBuf, responseLength);  // receive again to get the right data
  int retVal = -2;
  if (n >= responseLength && responseLength == 8) {
    retVal = fromHexHLConversion(recvBuf[5], recvBuf[6]);
  }
  else if (n > 4) {
    retVal = recvBuf[4];
  }
  return retVal;
}

void Dynamixel::WriteHeader(byte* buffer, byte length)
{
  //OXFF 0XFF ID LENGTH INSTRUCTION PARAMETER1 �PARAMETER N CHECK_SUM
  buffer[0] = 0xff;
  buffer[1] = 0xff;
  buffer[2] = _id;

  // bodyLength
  buffer[3] = length;
}

int Dynamixel::FormatCommand(byte command, byte address, std::vector<byte> values, byte* buffer)
{
  byte numberOfParameters = 0;

  WriteHeader(buffer);

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

  byte checksum = checkSumatory(buffer, 6 + numberOfParameters);
  buffer[6 + numberOfParameters] = checksum;

  return 9;
}

int Dynamixel::FormatCommand(byte command, byte address, byte* buffer)
{
  byte numberOfParameters = 0;

  WriteHeader(buffer, 4); // body is known to be 4 bytes long

  //the instruction
  buffer[4] = command;

  // start of goal registers
  buffer[5] = address;

  //bytes to read
  buffer[6] = 2;

  byte checksum = checkSum(buffer, 7);
  buffer[7] = checksum;

  return 9;
}

int Dynamixel::getPosition() 
{
  int ret=0;
  byte sendBuf[BufferSize] = {0};
  ret = FormatCommand(Commands["Get"],
		      Addresses["Position"],
		      sendBuf);

  ret = SendReceive(sendBuf, ret, CommandResponseLength["Get"]);
  return ret;
}

int Dynamixel::setPosition(int position) 
{
  int ret = 0;
  byte sendBuf[BufferSize] = {0};
  byte posH, posL;
  toHexHLConversion(position, &posH, &posL);
  std::vector<byte> data = {speedH, speedL};
  ret = FormatCommand(Commands["Set"],
		      Addresses["Goal"],
		      data,
		      sendBuf);
  ret = SendReceive(sendBuf, ret, CommandResponseLength["Set"]);
  return ret;
}

int Dynamixel::setSpeed(SerialPort *serialPort, int idAX12, int speed) 
{
  int ret = 0;
  byte sendBuf[BufferSize] = {0};
  byte speedH, speedL;
  toHexHLConversion(speed, &speedH, &speedL);
  std::vector<byte> data = {speedH, speedL};
  ret = FormatCommand(Commands["Set"],
		      Addresses["Speed"],
		      data,
		      sendBuf);
  ret = SendReceive(sendBuf, ret, CommandResponseLength["Set"]);
  return ret;
}

int Dynamixel::setCWAngleLimit(int limit) 
{
  int ret = 0;
  byte sendBuf[BufferSize] = {0};
  byte limitH, limitL;
  toHexHLConversion(limit, &limitH, &limitL);
  std::vector<byte> data = {limitH, limitL};
  ret = FormatCommand(Commands["Set"],
		      Addresses["CWAngleLimit"],
		      data,
		      sendBuf);
  ret = SendReceive(sendBuf, ret, CommandResponseLength["Set"]);
  return ret;
}

int Dynamixel::setCCWAngleLimit(int limit) 
{
  int ret = 0;
  byte sendBuf[BufferSize] = {0};
  byte limitH, limitL;
  toHexHLConversion(limit, &limitH, &limitL);
  std::vector<byte> data = {limitH, limitL};
  ret = FormatCommand(Commands["Set"],
		      Addresses["CCWAngleLimit"],
		      data,
		      sendBuf);
  ret = SendReceive(sendBuf, ret, CommandResponseLength["Set"]);
  return ret;
}

int Dynamixel::setCWComplianceMargin(int margin) 
{
  int ret = 0;
  byte sendBuf[BufferSize] = {0};
  std::vector<byte> data = {margin};
  ret = FormatCommand(Commands["Set"],
		      Addresses["CWComplianceMargin"],
		      data,
		      sendBuf);
  ret = SendReceive(sendBuf, ret, CommandResponseLength["Set"]);
  return ret;
}

int Dynamixel::setCCWComplianceMargin(int margin) 
{
  int ret = 0;
  byte sendBuf[BufferSize] = {0};
  std::vector<byte> data = {margin};
  ret = FormatCommand(Commands["Set"],
		      Addresses["CCWComplianceMargin"],
		      data,
		      sendBuf);
  ret = SendReceive(sendBuf, ret, CommandResponseLength["Set"]);
  return ret;
}

int Dynamixel::setCWComplianceSlope(SerialPort *serialPort, int idAX12, int slope) 
{
  int ret = 0;
  byte sendBuf[BufferSize] = {0};
  std::vector<byte> data = {margin};
  ret = FormatCommand(Commands["Set"],
		      Addresses["CWComplianceSlope"],
		      data,
		      sendBuf);
  ret = SendReceive(sendBuf, ret, CommandResponseLength["Set"]);
  return ret;
}

int Dynamixel::setCCWComplianceSlope(SerialPort *serialPort, int idAX12, int slope) 
{
  int ret = 0;
  byte sendBuf[BufferSize] = {0};
  std::vector<byte> data = {margin};
  ret = FormatCommand(Commands["Set"],
		      Addresses["CCWComplianceSlope"],
		      data,
		      sendBuf);
  ret = SendReceive(sendBuf, ret, CommandResponseLength["Set"]);
  return ret;
}
