#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace io {

struct Measurement {
    double angle;    // Radians counterclockwise from the sensor's zero direction.
    double distance; // Metres; zero means that the sensor reported no return.
    std::uint8_t intensity;
};

struct ScanPacket {
    std::array<Measurement, 12> measurements;
    std::uint16_t timestamp; // Raw device timestamp; not a host clock.
};

class StlParser {
public:
    // Feed bytes in arrival order. Partial packets survive between calls.
    // Returns a packet only after its header, CRC and angle fields are valid.
    // A zero-distance measurement is preserved; callers must not map it as a hit.
    std::optional<ScanPacket> push(std::uint8_t byte);

private:
    std::optional<ScanPacket> decode() const;
    void recover_header();

    std::array<std::uint8_t, 47> frame_{};
    std::size_t used_ = 0;
};

} // namespace io
