#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>
#include <system_error>
#include <termios.h>

namespace io {

enum class ReadStatus {
    Data,
    Timeout,
    Disconnected,
    Error
};

struct ReadResult {
    ReadStatus status;
    std::size_t size = 0;
    std::error_code error{};
};

// Linux transport only. Packet framing and mapping belong to other modules.
class SerialPort {
public:
    // Opens read-only, configures 230400 baud / 8N1 and flushes stale input.
    // Throws std::system_error on failure; a failed open never leaks a descriptor.
    static SerialPort Open(const char* device);
    ~SerialPort();

    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    // Waits up to timeout. A signal interruption also returns Timeout so that
    // the application can check its stop flag without restarting a blocking read.
    // Empty buffers or timeouts outside [0, INT_MAX] milliseconds are invalid.
    ReadResult read_some(std::span<std::uint8_t> buffer, std::chrono::milliseconds timeout) const;

private:
    SerialPort(int fd, const termios& original);

    int fd_;
    termios original_;
};

} // namespace io
