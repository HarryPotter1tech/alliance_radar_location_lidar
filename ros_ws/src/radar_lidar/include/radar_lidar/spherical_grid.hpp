#pragma once

#include <cmath>
#include <cstdint>
#include <numbers>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>

#include "radar_lidar/types.hpp"

namespace radar::lidar {

/// @brief 球面网格预处理
/// 将点云按球面角度（方位角×俯仰角）分桶，每个网格只保留最远的点
/// 比简单 VoxelGrid 更好地保留墙面等远距离特征
class SphericalGrid {
public:
    /// @param grid_size_deg 角度分辨率（度），默认 0.1°
    explicit SphericalGrid(double grid_size_deg = 0.1)
        : grid_size_rad_(grid_size_deg * std::numbers::pi / 180.0) { }

    /// @brief 添加一帧点云到网格中
    void add(const types::PointCloud& points);

    /// @brief 提取网格中的最远点，清空内部状态
    [[nodiscard]] auto extract() -> types::PointCloud;

    /// @brief 清空网格
    void clear() { grid_map_.clear(); }

    /// @brief 当前网格中的点数
    [[nodiscard]] auto size() const -> size_t { return grid_map_.size(); }

private:
    struct GridCell {
        Eigen::Vector3d farthest_point { 0, 0, 0 };
        double max_distance_sq = -1.0;
    };

    double grid_size_rad_;
    std::unordered_map<std::uint64_t, GridCell> grid_map_;
};

} // namespace radar::lidar
