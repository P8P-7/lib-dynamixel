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
    flush(FlushType::Receive);

    return boost::asio::write(*port, boost::asio::buffer(data));
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
    boost::optional<boost::system::error_code> timerResult;
    TimerType timer(io);
    timer.expires_from_now(timeout);
    timer.async_wait([&timerResult](const boost::system::error_code &error) {
        timerResult = error;
    });

    boost::optional<boost::system::error_code> readResult;
    boost::asio::async_read(*port, buffer, [&readResult, &bytesReceived](const boost::system::error_code &error, size_t bytes) {
        readResult = error;
        bytesReceived = bytes;
    });

    io.reset();
    while (io.run_one()) {
        if (readResult) {
            timer.cancel();
        } else if (timerResult) {
            port->cancel();
        }
    }

    if (readResult && *readResult != boost::system::errc::success) {
        auto first = boost::asio::buffer_cast<unsigned char *>(buffer);
        auto last = first + bytesReceived;

        std::string bufferStr;
        for (auto it = first; it != last; ++it) {
            bufferStr += (boost::format("0x%02X ") % static_cast<int>(*it)).str();
        }
        BOOST_LOG_TRIVIAL(debug) << "Buffer: " << bufferStr << " Bytes received: " << bytesReceived << ", expected: "
                                 << boost::asio::buffer_size(buffer);

        throw boost::system::system_error(*readResult);
    }
}