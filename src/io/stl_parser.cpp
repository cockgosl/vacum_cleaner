#include "io/stl_parser.hpp"

#include <cmath>
#include <numbers>
#include <span>

namespace io {
namespace {

std::uint16_t read_u16(const std::uint8_t* bytes) {
    // Protocol integers are little-endian, regardless of the host architecture.
    return static_cast<std::uint16_t>(bytes[0] | (static_cast<std::uint16_t>(bytes[1]) << 8));
}

std::uint8_t crc8(std::span<const std::uint8_t> bytes) {
    std::uint8_t crc = 0;
    for (const auto byte : bytes) {
        crc ^= byte;
        for (int bit = 0; bit < 8; ++bit)
            crc = static_cast<std::uint8_t>((crc << 1) ^ ((crc & 0x80) ? 0x4D : 0));
    }
    return crc;
}

} // namespace

std::optional<ScanPacket> StlParser::push(std::uint8_t byte) {
    if (used_ == 0) {
        if (byte == 0x54)
            frame_[used_++] = byte;
        return std::nullopt;
    }
    if (used_ == 1 && byte != 0x2C) {
        // A repeated 0x54 can be the beginning of the next header.
        used_ = byte == 0x54 ? 1 : 0;
        return std::nullopt;
    }
    frame_[used_++] = byte;
    if (used_ != frame_.size())
        return std::nullopt;

    auto packet = decode();
    if (packet)
        used_ = 0;
    else
        recover_header();
    return packet;
}

std::optional<ScanPacket> StlParser::decode() const {
    // Bytes 0..45 are protected by the CRC stored in byte 46.
    if (crc8(std::span{frame_}.first(46)) != frame_[46])
        return std::nullopt;
    const auto start = read_u16(frame_.data() + 4);
    const auto end = read_u16(frame_.data() + 42);
    if (start >= 36000 || end >= 36000)
        return std::nullopt;

    // Angles use hundredths of a degree and increase clockwise. Interpolation
    // must cross 359 -> 0 through the short forward interval, not backwards.
    const double sweep = (static_cast<int>(end) - start + 36000) % 36000;
    ScanPacket packet{};
    for (std::size_t i = 0; i < packet.measurements.size(); ++i) {
        const double raw_angle = std::fmod(start + sweep * i / (packet.measurements.size() - 1), 36000.0);
        const auto offset = 6 + i * 3;
        packet.measurements[i] = {-raw_angle * std::numbers::pi / 18000.0, read_u16(frame_.data() + offset) / 1000.0,
                                  frame_[offset + 2]};
    }
    packet.timestamp = read_u16(frame_.data() + 44);
    return packet;
}

void StlParser::recover_header() {
    // A damaged frame can contain the beginning of a valid frame. Preserve
    // that suffix instead of discarding all 47 bytes and missing the packet.
    std::size_t start = 1;
    for (; start < used_; ++start)
        if (frame_[start] == 0x54 && (start + 1 == used_ || frame_[start + 1] == 0x2C))
            break;
    for (std::size_t i = start; i < used_; ++i)
        frame_[i - start] = frame_[i];
    used_ -= start;
}

} // namespace io
