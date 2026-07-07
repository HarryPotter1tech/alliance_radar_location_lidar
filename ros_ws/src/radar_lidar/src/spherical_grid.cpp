#include "radar_lidar/spherical_grid.hpp"

#include <cmath>
#include <cstdint>
#include <ranges>

static auto pack_key(int azimuth_idx, int elevation_idx) -> std::uint64_t {
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(azimuth_idx)) << 32)
        | static_cast<std::uint32_t>(elevation_idx);
}

namespace radar::lidar {

void SphericalGrid::add(const types::PointCloud& points) {
    if (grid_map_.empty()) {
        grid_map_.reserve(points.size());
    }
    for (const auto& point : points) {
        double azimuth = std::atan2(point.y(), point.x());
        double elevation =
            std::atan2(point.z(), std::sqrt(point.x() * point.x() + point.y() * point.y()));

        int azimuth_idx   = static_cast<int>(std::floor(azimuth / grid_size_rad_));
        int elevation_idx = static_cast<int>(std::floor(elevation / grid_size_rad_));

        const double distance_sq = point.squaredNorm();

        auto& cell = grid_map_[pack_key(azimuth_idx, elevation_idx)];
        if (distance_sq > cell.max_distance_sq) {
            cell.farthest_point  = point;
            cell.max_distance_sq = distance_sq;
        }
    }
}

auto SphericalGrid::extract() -> types::PointCloud {
    auto result = grid_map_ | std::views::values
        | std::views::filter([](const auto& cell) { return cell.max_distance_sq >= 0.0; })
        | std::views::transform([](const auto& cell) { return cell.farthest_point; })
        | std::ranges::to<types::PointCloud>();
    grid_map_.clear();
    return result;
}

} // namespace radar::lidar
