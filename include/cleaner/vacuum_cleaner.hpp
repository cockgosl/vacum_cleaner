#pragma once

#include "mapping/lidar.hpp"
#include "rooting/scheduler.hpp"

#include <chrono>
#include <optional>

namespace cleaner {

struct RobotSpecs {
    double radius; // Metres; the first model has a circular footprint.
    std::chrono::seconds autonomy;
};

enum class StepStatus {
    Planned,
    PlanningFailed,
    TimeLimitReached,
    InvalidElapsedTime
};

struct StepResult {
    StepStatus status;
    // Present only when the scheduler was called; includes its failure reason.
    std::optional<rooting::PlanResult> plan;
};

class VacuumCleaner {
public:
    // Validates specs and creates the owned map through Lidar::Create.
    static std::optional<VacuumCleaner> Create(RobotSpecs specs, std::size_t width, std::size_t height, double resolution);

    bool update_ray(double start_x, double start_y, double hit_x, double hit_y);
    const lidar::Grid& grid() const noexcept;
    const RobotSpecs& specs() const noexcept;

    // One planning tick, without movement or marking any area as cleaned.
    // The caller supplies elapsed time since the cleaning session started.
    StepResult run_step(lidar::CellIndex start, lidar::CellIndex goal, std::chrono::seconds elapsed) const;

private:
    VacuumCleaner(RobotSpecs specs, lidar::Lidar lidar);

    RobotSpecs specs_;
    rooting::Scheduler scheduler_;
    lidar::Lidar lidar_;
};

} // namespace cleaner
