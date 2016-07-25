#include <string>

#include "dynamixel/Dynamixel.h"

int main(int argc, char** argv) {
  int motorId = 1;
  std::string command = "Get";
  std::string address = "Position";
  std::string motorType = "MX28";
  std::string portName = "/dev/ttyO5";
  int baudRate = 9600;

  // parse command line args
  for (int i=1; i<argc; i++) {
    if (!strcmp(argv[i],"--baudRate")) {
      baudRate = atoi(argv[++i]);
    }
    else if (!strcmp(argv[i],"--motorId")) {
      motorId = atoi(argv[++i]);
    }
    else if (!strcmp(argv[i],"--command")) {
      command = argv[++i];
    }
    else if (!strcmp(argv[i],"--address")) {
      address = argv[++i];
    }
    else if (!strcmp(argv[i],"--portName")) {
      portName = argv[++i];
    }
    else if (!strcmp(argv[i],"--motorType")) {
      motorType = argv[++i];
    }
  }

  Dynamixel* motor;

  if (motorType == "AX12") {
    motor = new AX12();
  }
  else if (motorType == "MX28") {
    motor = new MX28();
  }
  else {
    std::cout << "Error: motor type not supported!\n";
    return -1;
  }

  motor->Configure();

  SerialPort port;
  if (port.connect((char *)portName.c_str(), baudRate) != 0) {

    if (command == "Get") {
    }
    else if (command == "Set") {
    }
    
  }

  return 0;
}
