#include "io/scan_assembler.hpp"

#include <cmath>
#include <numbers>
#include <utility>

namespace io {

std::optional<Scan> ScanAssembler::push(const Measurement& measurement) {
    if (!std::isfinite(measurement.angle) || measurement.angle > 0.0 || measurement.angle <= -2.0 * std::numbers::pi ||
        !std::isfinite(measurement.distance) || measurement.distance < 0.0)
        return std::nullopt;

    // Clockwise angles are negative: 359 -> 0 degrees becomes a positive jump
    // from approximately -2*pi to zero. Small reverse jitter is not a new turn.
    const bool crossed_zero = previous_angle_ && measurement.angle - *previous_angle_ > std::numbers::pi;
    previous_angle_ = measurement.angle;
    if (crossed_zero) {
        std::optional<Scan> completed;
        if (collecting_)
            completed = Scan{std::move(pending_)};

        // The moved-from vector remains valid; clear also handles startup or overflow.
        pending_.clear();
        collecting_ = true;
        pending_.push_back(measurement);
        return completed;
    }

    if (collecting_) {
        if (pending_.size() == max_measurements) {
            // Never emit a truncated scan as a complete revolution.
            pending_.clear();
            collecting_ = false;
        } else
            pending_.push_back(measurement);
    }
    return std::nullopt;
}

} // namespace io
