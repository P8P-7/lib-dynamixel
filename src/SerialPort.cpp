#include <cstring>
#include <sys/ioctl.h>

#include "dynamixel/SerialPort.h"

SerialPort::SerialPort() {
    port = new boost::asio::serial_port(io);
}

int SerialPort::connect() {
    port->open("/dev/serial0");
    port->set_option(boost::asio::serial_port_base::baud_rate(9600));
    return 1;
}

int SerialPort::connect(std::string &device, unsigned int baud) {
    try {
        port->open(device);
        port->set_option(boost::asio::serial_port_base::baud_rate(baud));
    } catch (const boost::system::system_error& e) {
        return 0;
    }

    return 1;
}

void SerialPort::disconnect() {
    port->close();
}

int SerialPort::sendArray(unsigned char *buffer, int len) {
    return boost::asio::write(*port,
                              boost::asio::buffer(buffer, len));
}

int SerialPort::getArray(unsigned char *buffer, int len) {
    char rcvChar;
    int i = 0;
    while (i < len && boost::asio::read(*port, boost::asio::buffer(&rcvChar, 1)) == 1) {
        buffer[i++] = rcvChar;
    }
    return i;
}

