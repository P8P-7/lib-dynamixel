#pragma once

#include <cstdio>

#include <boost/asio/serial_port.hpp>
#include <boost/asio.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/bind.hpp>

class SerialPort {
private:
    boost::asio::io_service io;
    boost::asio::serial_port *port;

public:
    SerialPort();

    int connect();

    int connect(char *device, int baud);

    void disconnect(void);

    int sendArray(unsigned char *buffer, int len);

    int getArray(unsigned char *buffer, int len);
};
