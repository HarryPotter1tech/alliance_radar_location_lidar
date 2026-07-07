#pragma once

#include <expected>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "radar_lidar/types.hpp"

namespace radar::lidar {

struct ClusterConfig {
    double cluster_tolerance = 0.25;
    int min_cluster_size     = 5;
    int max_cluster_size     = 1000;
};

struct ClusterResult {
    Eigen::Vector3d centroid { 0, 0, 0 };
    Eigen::Vector3d min_bound { 0, 0, 0 }; // AABB min
    Eigen::Vector3d max_bound { 0, 0, 0 }; // AABB max
    int point_count = 0;
};

/// @brief 欧氏聚类 + 质心 + AABB 边界框
/// 输出质心 + AABB 边界框
class ClusterStage {
public:
    explicit ClusterStage(ClusterConfig cfg);

    /// @brief 对动态点云执行聚类
    /// @param dynamic_points 动态点（来自 DynamicCloudStage）
    /// @return 聚类结果列表（每个聚类一个质心 + AABB）
    auto process(const types::PointCloud& dynamic_points)
        -> std::expected<std::vector<ClusterResult>, std::string>;

private:
    ClusterConfig cfg_;
};

} // namespace radar::lidar
