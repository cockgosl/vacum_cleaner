#include "io/serial_port.hpp"

#include <cerrno>
#include <climits>
#include <fcntl.h>
#include <poll.h>
#include <stdexcept>
#include <unistd.h>

namespace io {

SerialPort SerialPort::Open(const char* device) {
    if (device == nullptr || device[0] == '\0')
        throw std::invalid_argument("Serial device path is empty");
    const int fd = ::open(device, O_RDONLY | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0)
        throw std::system_error(errno, std::generic_category(), "open serial port");

    termios original{};
    if (tcgetattr(fd, &original) != 0) {
        const int error = errno;
        ::close(fd);
        throw std::system_error(error, std::generic_category(), "read serial settings");
    }
    auto settings = original;
    cfmakeraw(&settings);
    settings.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
    settings.c_cflag |= CS8 | CLOCAL | CREAD;
    settings.c_cc[VMIN] = 1;
    settings.c_cc[VTIME] = 0;
    if (cfsetispeed(&settings, B230400) != 0 || cfsetospeed(&settings, B230400) != 0 || tcsetattr(fd, TCSANOW, &settings) != 0 ||
        tcflush(fd, TCIFLUSH) != 0) {
        const int error = errno;
        tcsetattr(fd, TCSANOW, &original);
        ::close(fd);
        throw std::system_error(error, std::generic_category(), "configure serial port");
    }
    // C++17 guaranteed copy elision allows returning an owning, non-copyable port.
    return SerialPort(fd, original);
}

SerialPort::SerialPort(int fd, const termios& original) : fd_(fd), original_(original) {}

SerialPort::~SerialPort() {
    // Restoration may fail after unplugging; the descriptor still must be closed.
    tcsetattr(fd_, TCSANOW, &original_);
    ::close(fd_);
}

ReadResult SerialPort::read_some(std::span<std::uint8_t> buffer, std::chrono::milliseconds timeout) const {
    if (buffer.empty() || timeout.count() < 0 || timeout.count() > INT_MAX)
        throw std::invalid_argument("Invalid serial read buffer or timeout");
    pollfd event{fd_, POLLIN, 0};
    const int ready = ::poll(&event, 1, static_cast<int>(timeout.count()));
    if (ready == 0 || (ready < 0 && errno == EINTR))
        return {ReadStatus::Timeout};
    if (ready < 0)
        return {ReadStatus::Error, 0, {errno, std::generic_category()}};

    // Drain readable bytes before reporting hangup; the last packet may already
    // be buffered when the peer closes its end of the connection.
    if (event.revents & POLLIN) {
        const auto count = ::read(fd_, buffer.data(), buffer.size());
        if (count > 0)
            return {ReadStatus::Data, static_cast<std::size_t>(count)};
        if (count == 0)
            return {ReadStatus::Disconnected};
        if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)
            return {ReadStatus::Timeout};
        return {ReadStatus::Error, 0, {errno, std::generic_category()}};
    }
    if (event.revents & POLLNVAL)
        return {ReadStatus::Error, 0, std::make_error_code(std::errc::bad_file_descriptor)};
    return {ReadStatus::Disconnected};
}

} // namespace io
