#include "radar_fusion/fusion_node.hpp"

#include <algorithm>
#include <cmath>
#include <string_view>
#include <tuple>

#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace radar::fusion {

using PoseCov = geometry_msgs::msg::PoseWithCovarianceStamped;

namespace {

    struct AssociationCandidate {
        std::size_t track_idx;
        std::size_t measurement_idx;
        double distance_sq;
    };

    auto to_string(FusionMode mode) -> std::string_view {
        switch (mode) {
        case FusionMode::RADAR_ONLY:
            return "RADAR_ONLY";
        case FusionMode::RADAR_CAMERA:
            return "RADAR_CAMERA";
        case FusionMode::DEGRADED:
            return "DEGRADED";
        }
        return "UNKNOWN";
    }

} // namespace

FusionNode::FusionNode(const rclcpp::NodeOptions& options)
    : Node("radar_fusion_node", options) {

    this->declare_parameter("gate_distance", 1.0);
    this->declare_parameter("track_timeout_sec", 1.5);
    this->declare_parameter("min_hits_to_confirm", 3);
    this->declare_parameter("max_misses_before_delete", 2);
    this->declare_parameter("max_tracks", 20);
    this->declare_parameter("enable_camera_fusion", false);

    cfg_.gate_distance            = this->get_parameter("gate_distance").as_double();
    cfg_.track_timeout_sec        = this->get_parameter("track_timeout_sec").as_double();
    cfg_.min_hits_to_confirm      = this->get_parameter("min_hits_to_confirm").as_int();
    cfg_.max_misses_before_delete = this->get_parameter("max_misses_before_delete").as_int();
    cfg_.max_tracks               = this->get_parameter("max_tracks").as_int();
    cfg_.enable_camera_fusion     = this->get_parameter("enable_camera_fusion").as_bool();
    tracks_.reserve(static_cast<std::size_t>(cfg_.max_tracks));

    sub_lidar_pose_ = this->create_subscription<PoseCov>(
        "/lidar/pose", 10, [this](const PoseCov::SharedPtr msg) { on_lidar_pose(msg); });

    sub_cluster_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/lidar/cluster", 10,
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { on_cluster(msg); });

    if (cfg_.enable_camera_fusion) {
        sub_camera_detection_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/camera/detection", 10, [this](const geometry_msgs::msg::PoseArray::SharedPtr msg) {
                on_camera_detection(msg);
            });
    }

    pub_tracks_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("/fusion/tracks", 10);
    pub_fused_tracks_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("/fusion/fused_tracks", 10);
    pub_pose_ = this->create_publisher<PoseCov>("/localization/pose", 10);
    pub_status_ =
        this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("/localization/status", 10);
    update_fusion_mode(this->now().nanoseconds());

    RCLCPP_INFO(get_logger(), "radar_fusion ready. gate=%.1fm timeout=%.1fs", cfg_.gate_distance,
        cfg_.track_timeout_sec);
}

void FusionNode::on_lidar_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    publish_localization_pose(*msg);
    publish_status(rclcpp::Time(msg->header.stamp));
}

void FusionNode::on_camera_detection(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    auto stamp              = rclcpp::Time(msg->header.stamp);
    latest_camera_stamp_ns_ = stamp.nanoseconds();
    latest_camera_observations_.clear();
    latest_camera_observations_.reserve(msg->poses.size());

    std::vector<Eigen::Vector2d> measurements;
    measurements.reserve(msg->poses.size());
    for (const auto& detection_pose : msg->poses) {
        latest_camera_observations_.push_back(CameraObservation {
            .position   = detection_pose.position,
            .confidence = 1.0,
        });
        if (std::isfinite(detection_pose.position.x) && std::isfinite(detection_pose.position.y)) {
            measurements.emplace_back(detection_pose.position.x, detection_pose.position.y);
        }
    }

    update_fusion_mode(latest_camera_stamp_ns_);
    process_measurements(measurements, stamp.nanoseconds(), false);

    publish_tracks(tracks_, stamp);
    publish_fused_tracks(tracks_, stamp);
    publish_status(stamp);
}

void FusionNode::process_measurements(
    const std::vector<Eigen::Vector2d>& measurements, int64_t now_ns, bool mark_unmatched_tracks) {
    for (auto& track : tracks_) {
        track.predict(now_ns);
    }

    std::vector<bool> matched_tracks(tracks_.size(), false);
    std::vector<bool> matched_meas(measurements.size(), false);
    const double gate_distance_sq = cfg_.gate_distance * cfg_.gate_distance;
    std::vector<AssociationCandidate> candidates;
    candidates.reserve(tracks_.size() * measurements.size());

    for (size_t i = 0; i < tracks_.size(); ++i) {
        for (size_t j = 0; j < measurements.size(); ++j) {
            const double d_sq = tracks_[i].distance_squared_to(measurements[j]);
            if (d_sq < gate_distance_sq) {
                candidates.push_back({ i, j, d_sq });
            }
        }
    }

    std::sort(candidates.begin(), candidates.end(),
        [](const AssociationCandidate& lhs, const AssociationCandidate& rhs) {
            return lhs.distance_sq < rhs.distance_sq;
        });

    for (const auto& candidate : candidates) {
        if (matched_tracks[candidate.track_idx] || matched_meas[candidate.measurement_idx]) {
            continue;
        }

        tracks_[candidate.track_idx].update(
            measurements[candidate.measurement_idx], now_ns, cfg_.min_hits_to_confirm);
        matched_tracks[candidate.track_idx]     = true;
        matched_meas[candidate.measurement_idx] = true;
    }

    if (mark_unmatched_tracks) {
        for (size_t i = 0; i < tracks_.size(); ++i) {
            if (!matched_tracks[i]) {
                tracks_[i].mark_missed(cfg_.max_misses_before_delete);
            }
        }
    }

    for (size_t j = 0; j < measurements.size(); ++j) {
        if (matched_meas[j]) continue;
        if (tracks_.size() >= static_cast<size_t>(cfg_.max_tracks)) break;

        KalmanTracker new_track(next_track_id_++);
        new_track.update(measurements[j], now_ns, cfg_.min_hits_to_confirm);
        tracks_.push_back(new_track);
    }

    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                      [&](const KalmanTracker& t) {
                          return t.state().is_deleted()
                              || t.state().is_stale(now_ns, cfg_.track_timeout_sec);
                      }),
        tracks_.end());
}

void FusionNode::on_cluster(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    auto stamp  = rclcpp::Time(msg->header.stamp);
    auto now_ns = stamp.nanoseconds();
    update_fusion_mode(now_ns);

    std::vector<Eigen::Vector2d> measurements;
    measurements.reserve(msg->width * msg->height);

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y) {
        if (std::isfinite(*iter_x) && std::isfinite(*iter_y)) {
            measurements.emplace_back(*iter_x, *iter_y);
        }
    }

    process_measurements(measurements, now_ns, true);

    publish_tracks(tracks_, stamp);
    publish_fused_tracks(tracks_, stamp);
    publish_status(stamp);
}

void FusionNode::publish_tracks(
    const std::vector<KalmanTracker>& tracks, const rclcpp::Time& stamp) {
    visualization_msgs::msg::MarkerArray markers;

    for (size_t i = 0; i < tracks.size(); ++i) {
        const auto& s = tracks[i].state();
        if (!s.is_confirmed()) continue;

        // 轨迹位置（球体）
        visualization_msgs::msg::Marker m;
        m.header.stamp    = stamp;
        m.header.frame_id = "map";
        m.ns              = "tracks";
        m.id              = s.track_id;
        m.type            = visualization_msgs::msg::Marker::SPHERE;
        m.action          = visualization_msgs::msg::Marker::ADD;

        m.pose.position.x    = s.x(0);
        m.pose.position.y    = s.x(1);
        m.pose.position.z    = 0.5;
        m.pose.orientation.w = 1.0;

        m.scale.x = 0.3;
        m.scale.y = 0.3;
        m.scale.z = 0.3;

        // 颜色：confirmed=绿色，颜色根据 color 字段
        if (s.color == 0) { // blue
            m.color.b = 1.0f;
            m.color.a = 0.8f;
        } else if (s.color == 2) { // red
            m.color.r = 1.0f;
            m.color.a = 0.8f;
        } else {
            m.color.g = 1.0f;
            m.color.a = 0.8f;
        }

        m.lifetime = rclcpp::Duration::from_seconds(0.5);
        markers.markers.push_back(m);

        // 速度箭头
        visualization_msgs::msg::Marker arrow;
        arrow.header.stamp    = stamp;
        arrow.header.frame_id = "map";
        arrow.ns              = "velocity";
        arrow.id              = s.track_id;
        arrow.type            = visualization_msgs::msg::Marker::ARROW;
        arrow.action          = visualization_msgs::msg::Marker::ADD;

        geometry_msgs::msg::Point start, end;
        start.x      = s.x(0);
        start.y      = s.x(1);
        start.z      = 0.5;
        end.x        = s.x(0) + s.x(2) * 0.5;
        end.y        = s.x(1) + s.x(3) * 0.5;
        end.z        = 0.5;
        arrow.points = { start, end };

        arrow.scale.x = 0.05;
        arrow.scale.y = 0.1;
        arrow.scale.z = 0.1;

        arrow.color.r  = 1.0f;
        arrow.color.a  = 0.6f;
        arrow.lifetime = rclcpp::Duration::from_seconds(0.5);
        markers.markers.push_back(arrow);

        // track ID 文字
        visualization_msgs::msg::Marker text;
        text.header.stamp    = stamp;
        text.header.frame_id = "map";
        text.ns              = "track_id";
        text.id              = s.track_id;
        text.type            = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text.action          = visualization_msgs::msg::Marker::ADD;

        text.pose.position.x    = s.x(0);
        text.pose.position.y    = s.x(1);
        text.pose.position.z    = 1.0;
        text.pose.orientation.w = 1.0;

        text.scale.z  = 0.3;
        text.color.r  = 1.0f;
        text.color.g  = 1.0f;
        text.color.b  = 1.0f;
        text.color.a  = 1.0f;
        text.text     = std::to_string(s.track_id);
        text.lifetime = rclcpp::Duration::from_seconds(0.5);
        markers.markers.push_back(text);
    }

    pub_tracks_->publish(markers);
}

void FusionNode::publish_fused_tracks(
    const std::vector<KalmanTracker>& tracks, const rclcpp::Time& stamp) {
    auto fused_markers = visualization_msgs::msg::MarkerArray();
    for (const auto& track : tracks) {
        const auto& state = track.state();
        if (!state.is_confirmed()) {
            continue;
        }

        auto marker               = visualization_msgs::msg::Marker();
        marker.header.stamp       = stamp;
        marker.header.frame_id    = "map";
        marker.ns                 = "fused_tracks";
        marker.id                 = state.track_id;
        marker.type               = visualization_msgs::msg::Marker::SPHERE;
        marker.action             = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x    = state.x(0);
        marker.pose.position.y    = state.x(1);
        marker.pose.position.z    = 0.6;
        marker.pose.orientation.w = 1.0;
        marker.scale.x            = 0.2;
        marker.scale.y            = 0.2;
        marker.scale.z            = 0.2;
        marker.color.r            = 1.0f;
        marker.color.g            = fusion_mode_ == FusionMode::RADAR_CAMERA ? 0.6f : 0.2f;
        marker.color.b            = 1.0f;
        marker.color.a            = 0.9f;
        marker.lifetime           = rclcpp::Duration::from_seconds(0.5);
        fused_markers.markers.push_back(marker);
    }

    pub_fused_tracks_->publish(fused_markers);
}

void FusionNode::publish_localization_pose(
    const geometry_msgs::msg::PoseWithCovarianceStamped& pose) {
    pub_pose_->publish(pose);
}

void FusionNode::publish_status(const rclcpp::Time& stamp) {
    auto status    = diagnostic_msgs::msg::DiagnosticStatus();
    status.level   = (fusion_mode_ == FusionMode::DEGRADED)
        ? diagnostic_msgs::msg::DiagnosticStatus::WARN
        : diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.name    = "radar_fusion/status";
    status.message = std::string(to_string(fusion_mode_));

    auto add_value = [&status](const std::string& key, const std::string& value) {
        auto item  = diagnostic_msgs::msg::KeyValue();
        item.key   = key;
        item.value = value;
        status.values.push_back(item);
    };
    add_value("stamp_ns", std::to_string(stamp.nanoseconds()));
    add_value("mode", std::string(to_string(fusion_mode_)));
    add_value("camera_enabled", cfg_.enable_camera_fusion ? "true" : "false");
    add_value("camera_observations", std::to_string(latest_camera_observations_.size()));
    add_value("track_count", std::to_string(tracks_.size()));

    pub_status_->publish(status);
}

void FusionNode::update_fusion_mode(int64_t reference_stamp_ns) {
    if (!cfg_.enable_camera_fusion) {
        fusion_mode_ = FusionMode::RADAR_ONLY;
        return;
    }

    const auto timeout_ns   = static_cast<int64_t>(cfg_.camera_timeout_sec * 1e9);
    const bool camera_stale = latest_camera_observations_.empty()
        || (reference_stamp_ns - latest_camera_stamp_ns_) > timeout_ns;

    fusion_mode_ = camera_stale ? FusionMode::DEGRADED : FusionMode::RADAR_CAMERA;
}

} // namespace radar::fusion
