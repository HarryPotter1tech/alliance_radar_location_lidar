#pragma once

#include <deque>
#include <expected>
#include <memory>
#include <string>
#include <vector>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "radar_lidar/config.hpp"
#include "radar_lidar/types.hpp"

namespace radar::lidar {

struct DynamicCloudConfig {
    double distance_threshold = 0.1;
    int num_threads           = 12;
    int accumulate_frames     = 3;
    config::RoiBounds roi { .use_roi = true,
        .x_min                       = 0,
        .x_max                       = 30,
        .y_min                       = -15,
        .y_max                       = 15,
        .z_min                       = 0,
        .z_max                       = 1.4 };
};

/// @brief 动态点提取
/// 用 KdTree K=1 最近邻把扫描和地图对比，距离 > 阈值的点 = 动态点（障碍物）
/// 支持 ROI 裁剪 + 多帧累积，可独立于 ROS 使用
class DynamicCloudStage {
public:
    explicit DynamicCloudStage(DynamicCloudConfig cfg);

    /// @brief 设置地图点云（用于 KdTree 构建）
    void set_map(const pcl::PointCloud<pcl::PointXYZ>::Ptr& map_cloud);

    /// @brief 处理一帧扫描，返回动态点
    /// @param scan 已变换到地图坐标系的扫描点云
    auto process(const types::PointCloud& scan) -> std::expected<types::PointCloud, std::string>;

    /// @brief 当前累积的动态点（合并所有帧）
    [[nodiscard]] auto accumulated() const -> types::PointCloud;

    /// @brief 清空累积
    void clear() { frames_.clear(); }

private:
    DynamicCloudConfig cfg_;
    pcl::KdTreeFLANN<pcl::PointXYZ> kd_tree_;
    bool map_set_ = false;

    std::deque<types::PointCloud> frames_;

    // 每线程复用缓冲区（构造时按线程数一次性分配，避免每帧堆分配）
    int thread_count_ = 1;
    std::vector<types::PointCloud> thread_clouds_;
    std::vector<std::vector<int>> thread_indices_;
    std::vector<std::vector<float>> thread_dist_sq_;
};

} // namespace radar::lidar
