#pragma once

#include <boost/log/trivial.hpp>
#include <boost/thread.hpp>
#include <boost/asio.hpp>

namespace goliath::dynamixel {
    class SerialPort {
    public:
        /**
         * Construct a serial port without opening it.
         */
        SerialPort();

        /**
         * Destroys the serial port
         */
        ~SerialPort();

        /**
         * Create a serial connection with dynamixel actuators.
         * \param device the serial device to connect with
         * \param baud the baud rate speed
         * \return true if connected successfully; otherwise, false.
         */
        bool connect(const std::string &device, unsigned int baud);

        /**
         * Close the serial port.
         */
        void close();

        /**
         * Set the timeout on read operations.
         * \param t duration for the timeout.
         */
        void setTimeout(const boost::asio::steady_timer::duration &t);

        /**
         * Write the supplied data to a serial device.
         * \param data to be sent through the serial device
         * \return the number of characters transferred.
         * \throws boost::system::system_error if any error
         */
        size_t write(const std::vector<unsigned char> &data);

        /**
         * Read a certain amount of data from the serial device
         * \param size how much data to read
         * \return the receive buffer. It's empty if no data is available.
         * \throws boost::system::system_error if any error
         */
        std::vector<unsigned char> read(size_t size);

    private:
        using TimerType = boost::asio::steady_timer;
        boost::asio::io_service io;
        std::unique_ptr<boost::asio::serial_port> port;

        TimerType::duration timeout;

        /**
         * https://stackoverflow.com/a/25018876/1480019
         */
        template<typename MutableBufferSequence>
        void readWithTimeout(const MutableBufferSequence &buffers);
    };
}