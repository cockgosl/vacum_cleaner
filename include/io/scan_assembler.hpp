#pragma once

#include "io/stl_parser.hpp"

#include <cstddef>
#include <optional>
#include <vector>

namespace io {

struct Scan {
    // Measurements stay in arrival order, including zero-distance returns.
    std::vector<Measurement> measurements;
};

class ScanAssembler {
public:
    static constexpr std::size_t max_measurements = 2048;

    // Accepts the parser's angles in (-2*pi, 0], decreasing as the sensor rotates.
    // The first observed zero crossing starts accumulation; the next emits a scan.
    // The crossing measurement belongs to the new scan, not the completed one.
    // A forward jump greater than pi denotes a crossing. Large gaps, dropped
    // packets and angular coverage are not validated at this stage.
    // Non-finite/out-of-range angles and negative/non-finite distances are ignored.
    // Overflow discards the pending scan and waits for the next crossing to restart.
    std::optional<Scan> push(const Measurement& measurement);

private:
    std::optional<double> previous_angle_;
    std::vector<Measurement> pending_;
    bool collecting_ = false;
};

} // namespace io
