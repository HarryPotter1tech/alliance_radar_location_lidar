#pragma once

#include <string>

namespace radar::lidar::config {

/// @brief 轴对齐包围盒 (AABB) ROI 参数
/// 被 LocalizationConfig 和 DynamicCloudConfig 共享嵌入,避免裁剪逻辑重复
struct RoiBounds {
    bool use_roi = false;
    double x_min = 0, x_max = 30;
    double y_min = -15, y_max = 15;
    double z_min = 0, z_max = 7;
};

struct LocalizationConfig {
    double voxel_leaf_size   = 0.1;
    double max_corr_distance = 1.0;
    int max_iterations       = 48;
    double rotation_eps      = 0.03;
    double translation_eps   = 0.1;
    int num_threads          = 4;
    bool verbose             = false;

    bool use_spherical_grid   = true;
    double spherical_grid_deg = 0.1;
    int accumulate_frames     = 20;

    bool use_lock_strategy = true;
    double lock_fitness    = 0.2;

    // 外部初值：由离线标定提供 t_map_lidar 时启用, 跳过在线配准直接锁定
    // 六自由度以平移(米)+欧拉角(弧度)给出, 旋转顺序 Rz(yaw)*Ry(pitch)*Rx(roll)
    bool has_initial_pose = false;
    double initial_tx = 0, initial_ty = 0, initial_tz = 0;
    double initial_roll = 0, initial_pitch = 0, initial_yaw = 0;

    RoiBounds roi;
};

} // namespace radar::lidar::config
