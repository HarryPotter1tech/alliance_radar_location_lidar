#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "radar_fusion/kalman_tracker.hpp"

namespace radar::fusion {

struct FusionConfig {
    double gate_distance         = 1.0; // 数据关联门限（米）
    double track_timeout_sec     = 1.5; // 轨迹超时删除
    int min_hits_to_confirm      = 3;   // 确认轨迹所需命中次数
    int max_misses_before_delete = 2;
    int max_tracks               = 20; // 最大轨迹数
    bool enable_camera_fusion    = false;
    double camera_timeout_sec    = 1.5; // 相机检测数据超时判定为 DEGRADED
};

enum class FusionMode {
    RADAR_ONLY,
    RADAR_CAMERA,
    DEGRADED,
};

struct CameraObservation {
    geometry_msgs::msg::Point position;
    double confidence = 0.0;
};

class FusionNode : public rclcpp::Node {
public:
    explicit FusionNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions { });

private:
    void on_lidar_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void on_camera_detection(const geometry_msgs::msg::PoseArray::SharedPtr msg);

    void on_cluster(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    void publish_tracks(const std::vector<KalmanTracker>& tracks, const rclcpp::Time& stamp);
    void publish_fused_tracks(const std::vector<KalmanTracker>& tracks, const rclcpp::Time& stamp);
    void publish_localization_pose(const geometry_msgs::msg::PoseWithCovarianceStamped& pose);
    void publish_status(const rclcpp::Time& stamp);
    void update_fusion_mode(int64_t reference_stamp_ns);

    void process_measurements(const std::vector<Eigen::Vector2d>& measurements, int64_t now_ns,
        bool mark_unmatched_tracks);

    FusionConfig cfg_;
    FusionMode fusion_mode_ = FusionMode::RADAR_ONLY;
    std::vector<KalmanTracker> tracks_;
    std::vector<CameraObservation> latest_camera_observations_;
    int64_t latest_camera_stamp_ns_ = 0;
    int next_track_id_              = 0;

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_lidar_pose_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cluster_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_camera_detection_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_tracks_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_fused_tracks_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr pub_status_;
};

} // namespace radar::fusion
