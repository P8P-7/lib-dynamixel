#pragma once

#include <boost/log/trivial.hpp>
#include <boost/thread.hpp>
#include <boost/asio.hpp>

namespace goliath::dynamixel {
    class SerialPort {
    public:
        SerialPort();

        virtual ~SerialPort();

        bool connect();

        bool connect(const std::string &device, unsigned int baud);

        void close();

        /**
         * Set the timeout on read operations.
         */
        void setTimeout(const boost::posix_time::time_duration &t);

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
        boost::asio::io_service io;
        std::unique_ptr<boost::asio::serial_port> port;

        boost::posix_time::time_duration timeout;

        template<typename SyncReadStream, typename MutableBufferSequence>
        void readWithTimeout(SyncReadStream &s, const MutableBufferSequence &buffers,
                             const boost::asio::deadline_timer::duration_type &expiry_time);
    };
}