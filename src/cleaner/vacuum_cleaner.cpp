#include "cleaner/vacuum_cleaner.hpp"

#include <cmath>
#include <utility>

namespace cleaner {

std::optional<VacuumCleaner> VacuumCleaner::Create(RobotSpecs specs, std::size_t width, std::size_t height, double resolution) {
    if (!std::isfinite(specs.radius) || specs.radius <= 0.0 || specs.autonomy.count() <= 0)
        return std::nullopt;
    auto lidar = lidar::Lidar::Create(width, height, resolution);
    if (!lidar)
        return std::nullopt;
    return VacuumCleaner(specs, std::move(*lidar));
}

VacuumCleaner::VacuumCleaner(RobotSpecs specs, lidar::Lidar lidar) : specs_(specs), lidar_(std::move(lidar)) {}

bool VacuumCleaner::update_ray(double start_x, double start_y, double hit_x, double hit_y) {
    return lidar_.update_ray(start_x, start_y, hit_x, hit_y);
}

const lidar::Grid& VacuumCleaner::grid() const noexcept {
    return lidar_.grid();
}

const RobotSpecs& VacuumCleaner::specs() const noexcept {
    return specs_;
}

StepResult VacuumCleaner::run_step(lidar::CellIndex start, lidar::CellIndex goal, std::chrono::seconds elapsed) const {
    if (elapsed.count() < 0)
        return {StepStatus::InvalidElapsedTime, std::nullopt};
    if (elapsed >= specs_.autonomy)
        return {StepStatus::TimeLimitReached, std::nullopt};
    auto plan = scheduler_.plan(lidar_.grid(), start, goal, specs_.radius);
    const auto status = plan.status == rooting::PlanStatus::Success ? StepStatus::Planned : StepStatus::PlanningFailed;
    return {status, std::move(plan)};
}

} // namespace cleaner
