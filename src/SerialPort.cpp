#include "dynamixel/SerialPort.h"

SerialPort::SerialPort() : timeout(boost::posix_time::milliseconds(500)) {
    port = new boost::asio::serial_port(io);
}

int SerialPort::connect() {
    port->open("/dev/serial0");
    port->set_option(boost::asio::serial_port_base::baud_rate(9600));
    return 1;
}

int SerialPort::connect(const std::string &device, unsigned int baud) {
    try {
        port->open(device);
        port->set_option(boost::asio::serial_port_base::baud_rate(baud));
    } catch (const boost::system::system_error &e) {
        return 0;
    }

    return 1;
}

void SerialPort::disconnect() {
    port->close();
}

void SerialPort::setTimeout(const boost::posix_time::time_duration &t) {
    timeout = t;
}

size_t SerialPort::write(const std::vector<unsigned char> &data) {
    return boost::asio::write(*port, boost::asio::buffer(data));
}

std::vector<unsigned char> SerialPort::read(size_t size) {
    // Allocate a vector with the desired size
    std::vector<unsigned char> result(size);
    readWithTimeout(*port, boost::asio::buffer(result), timeout); // Fill it with values
    return result;
}

template<typename SyncReadStream, typename MutableBufferSequence>
void SerialPort::readWithTimeout(SyncReadStream &s, const MutableBufferSequence &buffers,
                                 const boost::asio::deadline_timer::duration_type &expiry_time) {
    boost::optional<boost::system::error_code> timer_result;
    boost::asio::deadline_timer timer(s.get_io_service());
    timer.expires_from_now(expiry_time);
    timer.async_wait([&timer_result](const boost::system::error_code &error) {
        timer_result.reset(error);
    });

    boost::optional<boost::system::error_code> read_result;
    boost::asio::async_read(s, buffers, [&read_result](const boost::system::error_code &error, size_t) {
        read_result.reset(error);
    });

    s.get_io_service().reset();
    while (s.get_io_service().run_one()) {
        if (read_result) {
            timer.cancel();
        } else if (timer_result) {
            s.cancel();
        }
    }

    if (*read_result) {
        throw boost::system::system_error(*read_result);
    }

}