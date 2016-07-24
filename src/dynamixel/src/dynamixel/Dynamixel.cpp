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

int Dynamixel::FormatCommand(byte command, byte address, std::vector<byte> values, byte* buffer)
{
  byte numberOfParameters = 0;

  //OXFF 0XFF ID LENGTH INSTRUCTION PARAMETER1 �PARAMETER N CHECK SUM
  buffer[0] = 0xff;
  buffer[1] = 0xff;
  buffer[2] = _id;

  // bodyLength
  buffer[3] = 0; //place holder

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

  //OXFF 0XFF ID LENGTH INSTRUCTION PARAMETER1 �PARAMETER N CHECK SUM
  buffer[0] = 0xff;
  buffer[1] = 0xff;
  buffer[2] = _id;

  // bodyLength
  buffer[3] = 4;

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

int Dynamixel::getSetCWComplianceMarginCommand(byte id, short margin)
{
  int pos = 0;
  byte numberOfParameters = 0;
  //OXFF 0XFF ID LENGTH INSTRUCTION PARAMETER1 �PARAMETER N CHECK SUM

  buffer[pos++] = 0xff;
  buffer[pos++] = 0xff;
  buffer[pos++] = id;

  // bodyLength
  buffer[pos++] = 0; //place holder

  //the instruction, query => 3
  buffer[pos++] = 3;

  // CW Compliance Margin
  buffer[pos++] = 0x1A;
  buffer[pos++] = margin;
  numberOfParameters++;

  // bodyLength
  buffer[3] = (byte)(numberOfParameters + 3);

  byte checksum = checkSumatory(buffer, pos);
  buffer[pos++] = checksum;

  return pos;
}

int Dynamixel::getSetCCWComplianceMarginCommand(byte id, short margin)
{
  int pos = 0;
  byte numberOfParameters = 0;
  //OXFF 0XFF ID LENGTH INSTRUCTION PARAMETER1 �PARAMETER N CHECK SUM

  buffer[pos++] = 0xff;
  buffer[pos++] = 0xff;
  buffer[pos++] = id;

  // bodyLength
  buffer[pos++] = 0; //place holder

  //the instruction, query => 3
  buffer[pos++] = 3;

  // CCW Compliance Margin
  buffer[pos++] = 0x1B;
  buffer[pos++] = margin;
  numberOfParameters++;

  // bodyLength
  buffer[3] = (byte)(numberOfParameters + 3);

  byte checksum = checkSumatory(buffer, pos);
  buffer[pos++] = checksum;

  return pos;
}

int Dynamixel::getSetCWComplianceSlopeCommand(byte id, short slope)
{
  int pos = 0;
  byte numberOfParameters = 0;
  //OXFF 0XFF ID LENGTH INSTRUCTION PARAMETER1 �PARAMETER N CHECK SUM

  buffer[pos++] = 0xff;
  buffer[pos++] = 0xff;
  buffer[pos++] = id;

  // bodyLength
  buffer[pos++] = 0; //place holder

  //the instruction, query => 3
  buffer[pos++] = 3;

  // CCW Compliance Margin
  buffer[pos++] = 0x1C;
  buffer[pos++] = slope;
  numberOfParameters++;

  // bodyLength
  buffer[3] = (byte)(numberOfParameters + 3);

  byte checksum = checkSumatory(buffer, pos);
  buffer[pos++] = checksum;

  return pos;
}

int Dynamixel::getSetCCWComplianceSlopeCommand(byte id, short slope)
{
  int pos = 0;
  byte numberOfParameters = 0;
  //OXFF 0XFF ID LENGTH INSTRUCTION PARAMETER1 �PARAMETER N CHECK SUM

  buffer[pos++] = 0xff;
  buffer[pos++] = 0xff;
  buffer[pos++] = id;

  // bodyLength
  buffer[pos++] = 0; //place holder

  //the instruction, query => 3
  buffer[pos++] = 3;

  // CCW Compliance Margin
  buffer[pos++] = 0x1D;
  buffer[pos++] = slope;
  numberOfParameters++;

  // bodyLength
  buffer[3] = (byte)(numberOfParameters + 3);

  byte checksum = checkSumatory(buffer, pos);
  buffer[pos++] = checksum;

  return pos;
}


int Dynamixel::getSetCWAngleLimitCommand(byte id, short limit)
{
  int pos = 0;
  byte numberOfParameters = 0;
  //OXFF 0XFF ID LENGTH INSTRUCTION PARAMETER1 �PARAMETER N CHECK SUM

  buffer[pos++] = 0xff;
  buffer[pos++] = 0xff;
  buffer[pos++] = id;

  // bodyLength
  buffer[pos++] = 0; //place holder

  //the instruction, query => 3
  buffer[pos++] = 3;

  // CW Compliance Margin
  buffer[pos++] = 0x06;

  byte hexH = 0;
  byte hexL = 0;
  toHexHLConversion(limit, &hexH, &hexL);
  buffer[pos++] = hexL;
  numberOfParameters++;
  buffer[pos++] = hexH;
  numberOfParameters++;

  // bodyLength
  buffer[3] = (byte)(numberOfParameters + 3);

  byte checksum = checkSumatory(buffer, pos);
  buffer[pos++] = checksum;

  return pos;
}
int Dynamixel::getSetCCWAngleLimitCommand(byte id, short limit)
{
  int pos = 0;
  byte numberOfParameters = 0;
  //OXFF 0XFF ID LENGTH INSTRUCTION PARAMETER1 �PARAMETER N CHECK SUM

  buffer[pos++] = 0xff;
  buffer[pos++] = 0xff;
  buffer[pos++] = id;

  // bodyLength
  buffer[pos++] = 0; //place holder

  //the instruction, query => 3
  buffer[pos++] = 3;

  // CW Compliance Margin
  buffer[pos++] = 0x08;

  byte hexH = 0;
  byte hexL = 0;
  toHexHLConversion(limit, &hexH, &hexL);
  buffer[pos++] = hexL;
  numberOfParameters++;
  buffer[pos++] = hexH;
  numberOfParameters++;

  // bodyLength
  buffer[3] = (byte)(numberOfParameters + 3);

  byte checksum = checkSumatory(buffer, pos);
  buffer[pos++] = checksum;

  return pos;
}

int Dynamixel::setCWAngleLimit(SerialPort *serialPort, int idAX12, int limit) 
{
  int error=0;

  int n=getSetCWAngleLimitCommand(idAX12, limit);
  //bf(buffer,n);
  long l=serialPort->sendArray(buffer,n);
  Utils::sleepMS(waitTimeForResponse);

  memset(bufferIn,0,BufferSize);
  n=serialPort->getArray(bufferIn, n);
  //bf(bufferIn,n);
  memset(bufferIn,0,BufferSize);
  n=serialPort->getArray(bufferIn, setResponseLength);
  //bf(bufferIn,setResponseLength);

  if (n>4 && bufferIn[4] == 0)
    printf("setCWAngleLimit: id=<%i> set at value=<%i>\n", idAX12, limit);
  else {
    error=-1;
    printf("setCWAngleLimit: id=<%i> error: <%i>\n", idAX12, bufferIn[4]);
  }

  return error;
}

int Dynamixel::setCCWAngleLimit(SerialPort *serialPort, int idAX12, int limit) 
{
  int error=0;

  int n=getSetCCWAngleLimitCommand(idAX12, limit);
  //bf(buffer,n);
  long l=serialPort->sendArray(buffer,n);
  Utils::sleepMS(waitTimeForResponse);

  memset(bufferIn,0,BufferSize);
  n=serialPort->getArray(bufferIn, n);
  //bf(bufferIn,n);
  memset(bufferIn,0,BufferSize);
  n=serialPort->getArray(bufferIn, setResponseLength);
  //bf(bufferIn,setResponseLength);

  if (n>4 && bufferIn[4] == 0)
    printf("setCCWAngleLimit: id=<%i> set at value=<%i>\n", idAX12, limit);
  else {
    error=-1;
    printf("setCCWAngleLimit: id=<%i> error: <%i>\n", idAX12, bufferIn[4]);
  }

  return error;
}

int Dynamixel::setCWComplianceMargin(SerialPort *serialPort, int idAX12, int margin) 
{
  int error=0;

  int n=getSetCWComplianceMarginCommand(idAX12, margin);
  //bf(buffer,n);
  long l=serialPort->sendArray(buffer,n);
  Utils::sleepMS(waitTimeForResponse);

  memset(bufferIn,0,BufferSize);
  n=serialPort->getArray(bufferIn, n);
  //bf(bufferIn,n);
  memset(bufferIn,0,BufferSize);
  n=serialPort->getArray(bufferIn, setResponseLength);
  //bf(bufferIn,setResponseLength);

  if (n>4 && bufferIn[4] == 0)
    printf("setCWComplianceMargin: id=<%i> set at value=<%i>\n", idAX12, margin);
  else {
    error=-1;
    printf("setCWComplianceMargin: id=<%i> error: <%i>\n", idAX12, bufferIn[4]);
  }

  return error;
}

int Dynamixel::setCCWComplianceMargin(SerialPort *serialPort, int idAX12, int margin) 
{
  int error=0;

  int n=getSetCCWComplianceMarginCommand(idAX12, margin);
  //bf(buffer,n);
  long l=serialPort->sendArray(buffer,n);
  Utils::sleepMS(waitTimeForResponse);

  memset(bufferIn,0,BufferSize);
  n=serialPort->getArray(bufferIn, n);
  //bf(bufferIn,n);
  memset(bufferIn,0,BufferSize);
  n=serialPort->getArray(bufferIn, setResponseLength);
  //bf(bufferIn,setResponseLength);

  if (n>4 && bufferIn[4] == 0)
    printf("setCCWComplianceMargin: id=<%i> set at value=<%i>\n", idAX12, margin);
  else {
    error=-1;
    printf("setCCWComplianceMargin: id=<%i> error: <%i>\n", idAX12, bufferIn[4]);
  }

  return error;
}

int Dynamixel::setCWComplianceSlope(SerialPort *serialPort, int idAX12, int slope) 
{
  int error=0;

  int n=getSetCWComplianceSlopeCommand(idAX12, slope);
  //bf(buffer,n);
  long l=serialPort->sendArray(buffer,n);
  Utils::sleepMS(waitTimeForResponse);

  memset(bufferIn,0,BufferSize);
  n=serialPort->getArray(bufferIn, n);
  //bf(bufferIn,n);
  memset(bufferIn,0,BufferSize);
  n=serialPort->getArray(bufferIn, setResponseLength);
  //bf(bufferIn,setResponseLength);

  if (n>4 && bufferIn[4] == 0)
    printf("setCWComplianceSlope: id=<%i> set at value=<%i>\n", idAX12, slope);
  else {
    error=-1;
    printf("setCWComplianceSlope: id=<%i> error: <%i>\n", idAX12, bufferIn[4]);
  }

  return error;
}

int Dynamixel::setCCWComplianceSlope(SerialPort *serialPort, int idAX12, int slope) 
{
  int error=0;

  int n=getSetCCWComplianceMarginCommand(idAX12, slope);
  //bf(buffer,n);
  long l=serialPort->sendArray(buffer,n);
  Utils::sleepMS(waitTimeForResponse);

  memset(bufferIn,0,BufferSize);
  n=serialPort->getArray(bufferIn, n);
  //bf(bufferIn,n);
  memset(bufferIn,0,BufferSize);
  n=serialPort->getArray(bufferIn, setResponseLength);
  //bf(bufferIn,setResponseLength);

  if (n>4 && bufferIn[4] == 0)
    printf("setCCWComplianceSlope: id=<%i> set at value=<%i>\n", idAX12, slope);
  else {
    error=-1;
    printf("setCCWComplianceSlope: id=<%i> error: <%i>\n", idAX12, bufferIn[4]);
  }

  return error;
}
