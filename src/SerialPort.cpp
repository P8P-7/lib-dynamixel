#include "dynamixel/SerialPort.h"

using namespace goliath::dynamixel;

SerialPort::SerialPort() : port(std::make_unique<boost::asio::serial_port>(io)),
                           timeout(std::chrono::milliseconds(50)) {
}

bool SerialPort::connect(const std::string &device, unsigned int baud) {
    try {
        port->open(device);
        port->set_option(boost::asio::serial_port_base::baud_rate(baud));
    } catch (const boost::system::system_error &e) {
        BOOST_LOG_TRIVIAL(warning) << "Couldn't connect to serial port " << e.what();
        return false;
    }

    return true;
}

void SerialPort::close() {
    port->close();
}

SerialPort::~SerialPort() {
    if (port->is_open()) {
        close();
    }
}

void SerialPort::setTimeout(const TimerType::duration &t) {
    timeout = t;
}

size_t SerialPort::write(const std::vector<unsigned char> &data) {
    boost::system::error_code errorCode;

    flush(FlushType::Receive);

    auto result =  boost::asio::write(*port, boost::asio::buffer(data));

    return result;
}

void SerialPort::flush(FlushType what) {
    if (::tcflush(port->native_handle(), static_cast<int>(what)) != 0) {
        throw boost::system::system_error(errno, boost::asio::error::get_system_category());
    }
}

std::vector<unsigned char> SerialPort::read(size_t size) {
    // Allocate a vector with the desired size
    std::vector<unsigned char> result(size);

    readWithTimeout(boost::asio::buffer(result)); // Fill it with values
    return result;
}

template<typename MutableBufferSequence>
void SerialPort::readWithTimeout(const MutableBufferSequence &buffer) {
    size_t bytesReceived = 0;
    boost::optional<boost::system::error_code> timer_result;
    TimerType timer(io);
    timer.expires_from_now(timeout);
    timer.async_wait([&timer_result](const boost::system::error_code &error) {
        timer_result = error;
    });

    boost::optional<boost::system::error_code> read_result;
    boost::asio::async_read(*port, buffer, [&read_result, &bytesReceived](const boost::system::error_code &error, size_t bytes) {
        read_result = error;
        bytesReceived = bytes;
    });

    io.reset();
    while (io.run_one()) {
        if (read_result) {
            timer.cancel();
        } else if (timer_result) {
            port->cancel();
        }
    }

    if (read_result && *read_result != boost::system::errc::success) {
        auto first = boost::asio::buffer_cast<unsigned char*>(buffer);
        auto last = first + bytesReceived;

        std::string bufferStr;
        for (auto it = first; it != last; ++it) {
            bufferStr += (boost::format("0x%02X ") % static_cast<int>(*it)).str();
        }
        BOOST_LOG_TRIVIAL(debug) << "Buffer: " << bufferStr << " Bytes received: " << bytesReceived << ", expected: " << boost::asio::buffer_size(buffer);

        throw boost::system::system_error(*read_result);
    }
}