#include "radar_lidar/frame_accumulator.hpp"

#include <ranges>

namespace radar::lidar {

void FrameAccumulator::push(types::PointCloud points) {
    frames_.push_back(std::move(points));
    while (frames_.size() > window_size_) {
        frames_.pop_front();
    }
}

auto FrameAccumulator::all_points() const -> types::PointCloud {
    return frames_ | std::views::join | std::ranges::to<types::PointCloud>();
}

} // namespace radar::lidar
