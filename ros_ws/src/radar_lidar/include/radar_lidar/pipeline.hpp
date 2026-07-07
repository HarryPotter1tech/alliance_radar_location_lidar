#pragma once

#include <memory>
#include <optional>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "radar_lidar/cluster.hpp"
#include "radar_lidar/config.hpp"
#include "radar_lidar/dynamic_cloud.hpp"
#include "radar_lidar/localization.hpp"
#include "radar_lidar/map_data.hpp"
#include "radar_lidar/types.hpp"

namespace radar::lidar {

class LidarPipeline : public rclcpp::Node {
public:
    LidarPipeline();

private:
    void on_scan(const sensor_msgs::msg::PointCloud2::SharedPtr& msg);
    void publish_pose(const types::PoseEstimate& pose, types::Timestamp stamp);
    void publish_dynamic_tf(const types::PoseEstimate& pose, types::Timestamp stamp);
    void publish_diagnostics(const types::PoseEstimate& pose, double elapsed_ms, uint64_t frame);
    void publish_dynamic(const types::PointCloud& dynamic_points, types::Timestamp stamp);
    void publish_clusters(const std::vector<ClusterResult>& clusters, types::Timestamp stamp);

    void transform_scan_to_map(const types::PointCloud& scan, const types::PoseEstimate& pose,
        types::PointCloud& transformed);

    /// @brief 尝试从 TF 树读取 Odin1 内置重定位输出的 map->scan.frame_id 变换
    /// 仅在 use_odin_relocalization_tf_ 启用时调用；重定位未成功前返回 nullopt，
    /// 由调用方回退到现有 GICP 路径（localization_.process），核心配准逻辑不变
    auto try_odin_relocalization_pose(const std::string& source_frame, const rclcpp::Time& stamp)
        -> std::optional<types::PoseEstimate>;

    std::shared_ptr<const MapData> map_;
    LocalizationStage localization_;
    DynamicCloudStage dynamic_stage_;
    ClusterStage cluster_stage_;

    std::string scan_topic_   = "/livox/lidar";
    std::string hardware_id_  = "livox_mid70";
    std::string output_frame_ = "map";
    bool detection_enabled_   = true;

    // Odin1 内置重定位 TF 作为可选主位姿源；GICP 始终保留作为重定位未成功时的回退
    bool use_odin_relocalization_tf_ = false;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_scan_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr pub_diag_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_dynamic_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_clusters_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_cluster_viz_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    uint64_t frame_count_ { 0 };
    bool was_locked_ { false };
    bool was_odin_relocalized_ { false };
};

} // namespace radar::lidar
