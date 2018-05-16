#include <string>
#include <iostream>
#include "dynamixel/Dynamixel.h"

int main(int argc, char **argv) {
    int motorId = 1;
    int numBytes = 2;
    int iData = 0;
    std::string command = "Set";
    std::string address = "Goal";
    std::string motorType = "AX12";
    std::string portName = "/dev/serial0";
    int baudRate = 1000000;

    // motor objects
    Dynamixel *motor;
    AX12 ax12;

    std::vector<byte> data;
    std::vector<byte> recvData;


    if (numBytes == 1) {
        data.push_back(iData);
    } else if (numBytes == 2) {
        byte h, l;
        Utils::convertToHL(iData, &h, &l);
        data.push_back(l);
        data.push_back(h);
    }

    SerialPort port;
    std::cout << "Connecting to: " <<
              portName << ":" << baudRate << std::endl;
    if (port.connect((char *) portName.c_str(), baudRate) != 0) {

        std::cout << "Success\n";


        ax12 = AX12(motorId, &port);
        motor = new AX12(motorId, &port);
        motor->configure();

        // For debugging only:
        motor->setWheelMode(true);
        byte buffer[1024];
        int length = motor->turn(data, true);

        std::cout << "buffer: " <<
                  Utils::printBuffer(buffer, length) << std::endl;
        // end for debugging

        int retVal;
        retVal = motor->sendReceiveCommand(command,
                                           address,
                                           data,
                                           &recvData);

        int recvVal = 0;
        if (recvData.size() == 1) {
            recvVal = recvData[0];
        } else if (recvData.size() == 2) {
            recvVal = Utils::convertFromHL(recvData[0], recvData[1]);
        }

        std::cout << "received: " <<
                  retVal << " : " << recvVal << std::endl;

        std::cout << "position: " <<
                  motor->getPosition() << std::endl;
    } else {
        std::cout << "Couldn't open " <<
                  portName << " at baudRate " <<
                  baudRate << std::endl;
        return -1;
    }

    return 0;
}
