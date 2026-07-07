#pragma once

#include <deque>
#include <vector>

#include "radar_lidar/types.hpp"

namespace radar::lidar {

/// @brief 滑动窗口帧累积器
/// 维护最近 N 帧点云，用于球面网格预处理时积累足够点密度
class FrameAccumulator {
public:
    /// @param window_size 滑动窗口大小（帧数），默认 20
    explicit FrameAccumulator(size_t window_size = 20)
        : window_size_(window_size) { }

    /// @brief 添加一帧，超过窗口大小自动弹出最旧帧
    void push(types::PointCloud points);

    /// @brief 获取所有累积帧的点（合并）
    [[nodiscard]] auto all_points() const -> types::PointCloud;

    /// @brief 当前累积帧数
    [[nodiscard]] auto size() const -> size_t { return frames_.size(); }

    /// @brief 清空
    void clear() { frames_.clear(); }

    /// @brief 设置窗口大小
    void set_window_size(size_t n) { window_size_ = n; }

private:
    size_t window_size_;
    std::deque<types::PointCloud> frames_;
};

} // namespace radar::lidar
