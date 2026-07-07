#pragma once

#include <cmath>
#include <ranges>

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "radar_lidar/config.hpp"
#include "radar_lidar/types.hpp"

namespace radar::lidar::geom {

/// @brief 判断点是否在 AABB 范围内
[[nodiscard]] inline auto in_roi_aabb(const Eigen::Vector3d& p, const config::RoiBounds& roi)
    -> bool {
    return p.x() >= roi.x_min && p.x() <= roi.x_max && p.y() >= roi.y_min && p.y() <= roi.y_max
        && p.z() >= roi.z_min && p.z() <= roi.z_max;
}

/// @brief 对点云做 AABB ROI 裁剪; use_roi=false 时原样返回
/// 替代 localization.cpp / dynamic_cloud.cpp 中重复的裁剪循环
[[nodiscard]] inline auto clip_roi_aabb(
    const types::PointCloud& points, const config::RoiBounds& roi) -> types::PointCloud {
    if (!roi.use_roi) {
        return points;
    }
    return points | std::views::filter([&roi](const auto& p) { return in_roi_aabb(p, roi); })
        | std::ranges::to<types::PointCloud>();
}

/// @brief 由 yaw(Z)+pitch(Y) 组装位姿 (roll=0)
/// 旋转顺序 Rz(yaw)*Ry(pitch), 与场地系 (+x 前, +z 上) 一致
[[nodiscard]] inline auto pose_from_yaw_pitch(
    const Eigen::Vector3d& translation, double yaw, double pitch) -> Eigen::Isometry3d {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation()     = translation;
    pose.linear()          = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()))
                                 .toRotationMatrix();
    return pose;
}

/// @brief look-at: 由观察点 eye 指向注视点 target, 反解 yaw+pitch (roll=0)
[[nodiscard]] inline auto look_at_yaw_pitch(
    const Eigen::Vector3d& eye, const Eigen::Vector3d& target) -> std::pair<double, double> {
    const Eigen::Vector3d d = (target - eye).normalized();
    const double yaw        = std::atan2(d.y(), d.x());
    const double pitch      = std::atan2(-d.z(), std::hypot(d.x(), d.y()));
    return { yaw, pitch };
}

/// @brief 过滤 PCL 点云中的无效点 (非有限值 / 零范数), 转为 Eigen 表示的 Frame
/// 替代 pipeline.cpp / offline_test_node.cpp / registration_tool.cpp 中重复的过滤逻辑
[[nodiscard]] inline auto filter_valid_points(const pcl::PointCloud<pcl::PointXYZ>& pcl_cloud)
    -> types::Frame {
    auto points = pcl_cloud.points | std::views::filter([](const auto& pt) {
        return std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)
            && (pt.x * pt.x + pt.y * pt.y + pt.z * pt.z) > 1e-12;
    }) | std::views::transform([](const auto& pt) { return Eigen::Vector3d(pt.x, pt.y, pt.z); })
        | std::ranges::to<types::PointCloud>();
    return { .points = std::move(points) };
}

} // namespace radar::lidar::geom
