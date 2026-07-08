#pragma once

#include <cmath>
#include <cstdint>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "radar_lidar/types.hpp"

namespace radar::lidar::offline {

struct BgrColor {
    std::uint8_t b;
    std::uint8_t g;
    std::uint8_t r;
};

inline constexpr BgrColor kMapColorBgr { 255, 255, 0 };
inline constexpr BgrColor kRawScanColorBgr { 160, 48, 0 };
inline constexpr BgrColor kAlignedScanColorBgr { 0, 255, 0 };
inline constexpr BgrColor kRoiColorBgr { 128, 255, 128 };

inline auto is_valid_xyz(float x, float y, float z) -> bool {
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

inline auto is_valid_xyz(const Eigen::Vector3d& pt) -> bool {
    return std::isfinite(pt.x()) && std::isfinite(pt.y()) && std::isfinite(pt.z());
}

inline auto make_colored_point(float x, float y, float z, BgrColor color) -> pcl::PointXYZRGB {
    auto point = pcl::PointXYZRGB();
    point.x    = x;
    point.y    = y;
    point.z    = z;
    point.b    = color.b;
    point.g    = color.g;
    point.r    = color.r;
    point.a    = 255;
    return point;
}

[[nodiscard]] inline auto make_colored_cloud(const pcl::PointCloud<pcl::PointXYZ>& input,
    BgrColor color) -> pcl::PointCloud<pcl::PointXYZRGB> {
    pcl::PointCloud<pcl::PointXYZRGB> output;
    output.reserve(input.size());
    for (const auto& pt : input.points) {
        if (!is_valid_xyz(pt.x, pt.y, pt.z)) continue;
        output.push_back(make_colored_point(pt.x, pt.y, pt.z, color));
    }
    output.width    = output.size();
    output.height   = 1;
    output.is_dense = true;
    return output;
}

[[nodiscard]] inline auto make_colored_cloud(const radar::lidar::types::PointCloud& input,
    BgrColor color) -> pcl::PointCloud<pcl::PointXYZRGB> {
    pcl::PointCloud<pcl::PointXYZRGB> output;
    output.reserve(input.size());
    for (const auto& pt : input) {
        if (!is_valid_xyz(pt)) continue;
        output.push_back(make_colored_point(static_cast<float>(pt.x()), static_cast<float>(pt.y()),
            static_cast<float>(pt.z()), color));
    }
    output.width    = output.size();
    output.height   = 1;
    output.is_dense = true;
    return output;
}

[[nodiscard]] inline auto make_overlay_cloud(const pcl::PointCloud<pcl::PointXYZ>& map_cloud,
    const radar::lidar::types::PointCloud& aligned_scan, BgrColor map_color, BgrColor scan_color)
    -> pcl::PointCloud<pcl::PointXYZRGB> {
    auto output = make_colored_cloud(map_cloud, map_color);
    output.reserve(output.size() + aligned_scan.size());
    for (const auto& pt : aligned_scan) {
        if (!is_valid_xyz(pt)) continue;
        output.push_back(make_colored_point(static_cast<float>(pt.x()), static_cast<float>(pt.y()),
            static_cast<float>(pt.z()), scan_color));
    }
    output.width    = output.size();
    output.height   = 1;
    output.is_dense = true;
    return output;
}

} // namespace radar::lidar::offline
