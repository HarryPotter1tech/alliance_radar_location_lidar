#include <gtest/gtest.h>

#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
#include <tuple>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "radar_fusion/fusion_node.hpp"

namespace {

using namespace std::chrono_literals;

class FusionNodeTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() {
        if (!rclcpp::ok()) {
            int argc = 0;
            rclcpp::init(argc, nullptr);
        }
    }

    static void TearDownTestSuite() {
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
    }

    void SetUp() override {
        fusion_node_     = std::make_shared<radar::fusion::FusionNode>();
        publisher_node_  = std::make_shared<rclcpp::Node>("fusion_node_test_publisher");
        subscriber_node_ = std::make_shared<rclcpp::Node>("fusion_node_test_subscriber");

        cluster_pub_ =
            publisher_node_->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar/cluster", 10);
        lidar_pose_pub_ =
            publisher_node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
                "/lidar/pose", 10);
        camera_detection_pub_ = publisher_node_->create_publisher<geometry_msgs::msg::PoseArray>(
            "/camera/detection", 10);

        pose_sub_ =
            subscriber_node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                "/localization/pose", 10,
                [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
                    std::lock_guard<std::mutex> lock(mutex_);
                    last_pose_ = *msg;
                    ++pose_count_;
                    cv_.notify_all();
                });
        tracks_sub_ =
            subscriber_node_->create_subscription<visualization_msgs::msg::MarkerArray>("/fusion/"
                                                                                        "tracks",
                10, [this](const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
                    std::lock_guard<std::mutex> lock(mutex_);
                    last_track_marker_count_ = msg->markers.size();
                    ++track_publish_count_;
                    cv_.notify_all();
                });
        fused_tracks_sub_ =
            subscriber_node_->create_subscription<visualization_msgs::msg::MarkerArray>("/fusion/"
                                                                                        "fused_"
                                                                                        "tracks",
                10, [this](const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
                    std::lock_guard<std::mutex> lock(mutex_);
                    last_fused_track_marker_count_ = msg->markers.size();
                    ++fused_track_publish_count_;
                    cv_.notify_all();
                });
        status_sub_ =
            subscriber_node_->create_subscription<diagnostic_msgs::msg::DiagnosticStatus>("/localiz"
                                                                                          "ation/"
                                                                                          "status",
                10, [this](const diagnostic_msgs::msg::DiagnosticStatus::SharedPtr msg) {
                    std::lock_guard<std::mutex> lock(mutex_);
                    last_status_ = *msg;
                    ++status_count_;
                    cv_.notify_all();
                });

        executor_.add_node(fusion_node_);
        executor_.add_node(publisher_node_);
        executor_.add_node(subscriber_node_);
        spin_thread_ = std::thread([this]() { executor_.spin(); });

        ASSERT_TRUE(wait_for_discovery()) << "ROS entities failed to discover each other";
    }

    void TearDown() override {
        executor_.cancel();
        if (spin_thread_.joinable()) {
            spin_thread_.join();
        }
        executor_.remove_node(subscriber_node_);
        executor_.remove_node(publisher_node_);
        executor_.remove_node(fusion_node_);

        pose_sub_.reset();
        tracks_sub_.reset();
        fused_tracks_sub_.reset();
        status_sub_.reset();
        cluster_pub_.reset();
        lidar_pose_pub_.reset();
        camera_detection_pub_.reset();
        subscriber_node_.reset();
        publisher_node_.reset();
        fusion_node_.reset();

        {
            std::lock_guard<std::mutex> lock(mutex_);
            pose_count_                    = 0;
            track_publish_count_           = 0;
            last_track_marker_count_       = 0;
            fused_track_publish_count_     = 0;
            last_fused_track_marker_count_ = 0;
            status_count_                  = 0;
            last_pose_                     = geometry_msgs::msg::PoseWithCovarianceStamped();
            last_status_                   = diagnostic_msgs::msg::DiagnosticStatus();
        }
    }

    auto wait_for_discovery(bool expect_camera = false) -> bool {
        const auto deadline = std::chrono::steady_clock::now() + 2s;
        while (std::chrono::steady_clock::now() < deadline) {
            if (cluster_pub_->get_subscription_count() > 0
                && lidar_pose_pub_->get_subscription_count() > 0
                && pose_sub_->get_publisher_count() > 0 && tracks_sub_->get_publisher_count() > 0
                && fused_tracks_sub_->get_publisher_count() > 0
                && status_sub_->get_publisher_count() > 0
                && (!expect_camera || camera_detection_pub_->get_subscription_count() > 0)) {
                return true;
            }
            std::this_thread::sleep_for(20ms);
        }
        return false;
    }

    auto wait_for_pose_count(std::size_t expected_count, std::chrono::milliseconds timeout)
        -> bool {
        std::unique_lock<std::mutex> lock(mutex_);
        return cv_.wait_for(lock, timeout, [&]() { return pose_count_ >= expected_count; });
    }

    auto wait_for_track_marker_count(
        std::size_t expected_marker_count, std::chrono::milliseconds timeout) -> bool {
        std::unique_lock<std::mutex> lock(mutex_);
        return cv_.wait_for(
            lock, timeout, [&]() { return last_track_marker_count_ >= expected_marker_count; });
    }

    auto wait_for_track_publish_count(
        std::size_t expected_publish_count, std::chrono::milliseconds timeout) -> bool {
        std::unique_lock<std::mutex> lock(mutex_);
        return cv_.wait_for(
            lock, timeout, [&]() { return track_publish_count_ >= expected_publish_count; });
    }

    auto wait_for_fused_track_publish_count(
        std::size_t expected_publish_count, std::chrono::milliseconds timeout) -> bool {
        std::unique_lock<std::mutex> lock(mutex_);
        return cv_.wait_for(
            lock, timeout, [&]() { return fused_track_publish_count_ >= expected_publish_count; });
    }

    auto wait_for_status_count(std::size_t expected_count, std::chrono::milliseconds timeout)
        -> bool {
        std::unique_lock<std::mutex> lock(mutex_);
        return cv_.wait_for(lock, timeout, [&]() { return status_count_ >= expected_count; });
    }

    auto make_cluster_msg(double x, double y, double z, int32_t sec, uint32_t nanosec)
        -> sensor_msgs::msg::PointCloud2 {
        return make_cluster_array_msg({ { x, y, z } }, sec, nanosec);
    }

    auto make_cluster_array_msg(const std::vector<std::tuple<double, double, double>>& points,
        int32_t sec, uint32_t nanosec) -> sensor_msgs::msg::PointCloud2 {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        cloud->reserve(points.size());
        for (const auto& [x, y, z] : points) {
            cloud->emplace_back(
                static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
        }
        cloud->width    = cloud->size();
        cloud->height   = 1;
        cloud->is_dense = true;

        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*cloud, msg);
        msg.header.stamp.sec     = sec;
        msg.header.stamp.nanosec = nanosec;
        msg.header.frame_id      = "map";
        return msg;
    }

    auto make_empty_cluster_msg(int32_t sec, uint32_t nanosec) -> sensor_msgs::msg::PointCloud2 {
        return make_cluster_array_msg({ }, sec, nanosec);
    }

    rclcpp::executors::SingleThreadedExecutor executor_;
    std::shared_ptr<radar::fusion::FusionNode> fusion_node_;
    rclcpp::Node::SharedPtr publisher_node_;
    rclcpp::Node::SharedPtr subscriber_node_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cluster_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr lidar_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr camera_detection_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr tracks_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr fused_tracks_sub_;
    rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr status_sub_;
    std::thread spin_thread_;

    std::mutex mutex_;
    std::condition_variable cv_;
    std::size_t pose_count_                    = 0;
    std::size_t track_publish_count_           = 0;
    std::size_t last_track_marker_count_       = 0;
    std::size_t fused_track_publish_count_     = 0;
    std::size_t last_fused_track_marker_count_ = 0;
    std::size_t status_count_                  = 0;
    geometry_msgs::msg::PoseWithCovarianceStamped last_pose_;
    diagnostic_msgs::msg::DiagnosticStatus last_status_;
};

}

TEST_F(FusionNodeTest, ClusterOnlyInputPublishesStatusWithoutLocalizationPose) {
    auto cluster_msg = make_cluster_msg(1.0, 2.0, 0.0, 0, 123456789u);
    cluster_pub_->publish(cluster_msg);

    EXPECT_FALSE(wait_for_pose_count(1, 300ms));
    EXPECT_TRUE(wait_for_status_count(1, 300ms));
}

TEST_F(FusionNodeTest, ClusterTrackingUsesMessageTimeInsteadOfWallTime) {
    cluster_pub_->publish(make_cluster_msg(0.0, 0.0, 0.0, 0, 0u));
    std::this_thread::sleep_for(1000ms);
    cluster_pub_->publish(make_cluster_msg(0.8, 0.0, 0.0, 1, 0u));
    std::this_thread::sleep_for(1000ms);
    cluster_pub_->publish(make_cluster_msg(1.6, 0.0, 0.0, 2, 0u));

    ASSERT_TRUE(wait_for_track_marker_count(3, 500ms));

    {
        std::lock_guard<std::mutex> lock(mutex_);
        track_publish_count_     = 0;
        last_track_marker_count_ = 0;
    }

    std::this_thread::sleep_for(1700ms);
    cluster_pub_->publish(make_cluster_msg(1.68, 0.0, 0.0, 2, 100000000u));

    ASSERT_TRUE(wait_for_track_publish_count(1, 500ms));

    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_EQ(last_track_marker_count_, 3u);
}

TEST_F(FusionNodeTest, TentativeTrackIsDroppedAfterSingleMiss) {
    cluster_pub_->publish(make_cluster_msg(0.0, 0.0, 0.0, 0, 0u));
    std::this_thread::sleep_for(200ms);

    auto empty_msg = make_empty_cluster_msg(0, 100000000u);
    cluster_pub_->publish(empty_msg);
    std::this_thread::sleep_for(200ms);

    ASSERT_FALSE(wait_for_track_marker_count(3, 200ms));
}

TEST_F(FusionNodeTest, ConfirmedTrackSurvivesSingleMissButDropsAfterSecondMiss) {
    cluster_pub_->publish(make_cluster_msg(0.0, 0.0, 0.0, 0, 0u));
    std::this_thread::sleep_for(1000ms);
    cluster_pub_->publish(make_cluster_msg(0.8, 0.0, 0.0, 1, 0u));
    std::this_thread::sleep_for(1000ms);
    cluster_pub_->publish(make_cluster_msg(1.6, 0.0, 0.0, 2, 0u));

    ASSERT_TRUE(wait_for_track_marker_count(3, 500ms));

    {
        std::lock_guard<std::mutex> lock(mutex_);
        track_publish_count_     = 0;
        last_track_marker_count_ = 0;
    }

    auto first_miss = make_empty_cluster_msg(2, 100000000u);
    cluster_pub_->publish(first_miss);

    ASSERT_TRUE(wait_for_track_publish_count(1, 500ms));
    {
        std::lock_guard<std::mutex> lock(mutex_);
        EXPECT_EQ(last_track_marker_count_, 3u);
        track_publish_count_     = 0;
        last_track_marker_count_ = 0;
    }

    auto second_miss = make_empty_cluster_msg(2, 200000000u);
    cluster_pub_->publish(second_miss);

    ASSERT_TRUE(wait_for_track_publish_count(1, 500ms));
    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_EQ(last_track_marker_count_, 0u);
}

TEST_F(FusionNodeTest, GlobalGreedyAssociationKeepsTwoConfirmedTracks) {
    cluster_pub_->publish(make_cluster_array_msg({ { 0.0, 0.0, 0.0 }, { 1.5, 0.0, 0.0 } }, 0, 0u));
    std::this_thread::sleep_for(1000ms);

    cluster_pub_->publish(make_cluster_array_msg({ { 0.4, 0.0, 0.0 }, { 1.9, 0.0, 0.0 } }, 1, 0u));
    std::this_thread::sleep_for(1000ms);

    cluster_pub_->publish(make_cluster_array_msg({ { 0.8, 0.0, 0.0 }, { 2.3, 0.0, 0.0 } }, 2, 0u));

    ASSERT_TRUE(wait_for_track_marker_count(6, 500ms));

    {
        std::lock_guard<std::mutex> lock(mutex_);
        track_publish_count_     = 0;
        last_track_marker_count_ = 0;
    }

    cluster_pub_->publish(make_cluster_array_msg({ { 1.1, 0.0, 0.0 }, { 1.2, 0.0, 0.0 } }, 3, 0u));

    ASSERT_TRUE(wait_for_track_publish_count(1, 500ms));
    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_EQ(last_track_marker_count_, 6u);
}

TEST_F(FusionNodeTest, LidarPoseIsForwardedToLocalizationPose) {
    geometry_msgs::msg::PoseWithCovarianceStamped lidar_pose;
    lidar_pose.header.stamp.sec        = 0;
    lidar_pose.header.stamp.nanosec    = 987654321u;
    lidar_pose.header.frame_id         = "map";
    lidar_pose.pose.pose.position.x    = 1.25;
    lidar_pose.pose.pose.position.y    = -0.75;
    lidar_pose.pose.pose.position.z    = 0.5;
    lidar_pose.pose.pose.orientation.x = 0.1;
    lidar_pose.pose.pose.orientation.y = -0.2;
    lidar_pose.pose.pose.orientation.z = 0.3;
    lidar_pose.pose.pose.orientation.w = 0.9;

    lidar_pose_pub_->publish(lidar_pose);
    ASSERT_TRUE(wait_for_pose_count(1, 1s));

    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_EQ(last_pose_.header.frame_id, lidar_pose.header.frame_id);
    EXPECT_EQ(last_pose_.header.stamp.sec, lidar_pose.header.stamp.sec);
    EXPECT_EQ(last_pose_.header.stamp.nanosec, lidar_pose.header.stamp.nanosec);
    EXPECT_DOUBLE_EQ(last_pose_.pose.pose.position.x, lidar_pose.pose.pose.position.x);
    EXPECT_DOUBLE_EQ(last_pose_.pose.pose.position.y, lidar_pose.pose.pose.position.y);
    EXPECT_DOUBLE_EQ(last_pose_.pose.pose.position.z, lidar_pose.pose.pose.position.z);
    EXPECT_DOUBLE_EQ(last_pose_.pose.pose.orientation.x, lidar_pose.pose.pose.orientation.x);
    EXPECT_DOUBLE_EQ(last_pose_.pose.pose.orientation.y, lidar_pose.pose.pose.orientation.y);
    EXPECT_DOUBLE_EQ(last_pose_.pose.pose.orientation.z, lidar_pose.pose.pose.orientation.z);
    EXPECT_DOUBLE_EQ(last_pose_.pose.pose.orientation.w, lidar_pose.pose.pose.orientation.w);
    EXPECT_DOUBLE_EQ(last_pose_.pose.covariance[0], lidar_pose.pose.covariance[0]);
}

TEST_F(FusionNodeTest, ConfirmedTracksArePublishedToFusedTracks) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        fused_track_publish_count_     = 0;
        last_fused_track_marker_count_ = 0;
    }

    cluster_pub_->publish(make_cluster_msg(0.0, 0.0, 0.0, 0, 0u));
    std::this_thread::sleep_for(1000ms);
    cluster_pub_->publish(make_cluster_msg(0.8, 0.0, 0.0, 1, 0u));
    std::this_thread::sleep_for(1000ms);
    cluster_pub_->publish(make_cluster_msg(1.6, 0.0, 0.0, 2, 0u));

    ASSERT_TRUE(wait_for_track_marker_count(3, 500ms));

    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_EQ(last_fused_track_marker_count_, 1u);
}

TEST_F(FusionNodeTest, LidarPosePublishesRadarOnlyStatus) {
    geometry_msgs::msg::PoseWithCovarianceStamped lidar_pose;
    lidar_pose.header.stamp.sec = 1;
    lidar_pose.header.frame_id  = "map";

    lidar_pose_pub_->publish(lidar_pose);
    ASSERT_TRUE(wait_for_status_count(1, 1s));

    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_EQ(last_status_.message, "RADAR_ONLY");
}

TEST_F(FusionNodeTest, CameraDetectionSwitchesFusionModeWhenEnabled) {
    auto enabled_node = std::make_shared<radar::fusion::FusionNode>(
        rclcpp::NodeOptions().append_parameter_override("enable_camera_fusion", true));
    executor_.remove_node(fusion_node_);
    fusion_node_.reset();
    fusion_node_ = enabled_node;
    executor_.add_node(fusion_node_);
    ASSERT_TRUE(wait_for_discovery(/*expect_camera=*/true)) << "ROS entities failed to rediscover "
                                                               "after enabling camera fusion";

    {
        std::lock_guard<std::mutex> lock(mutex_);
        status_count_ = 0;
    }

    geometry_msgs::msg::PoseArray detections;
    detections.header.frame_id  = "map";
    detections.header.stamp.sec = 2;
    geometry_msgs::msg::Pose pose;
    pose.position.x = 1.0;
    pose.position.y = 2.0;
    pose.position.z = 0.0;
    detections.poses.push_back(pose);
    camera_detection_pub_->publish(detections);

    ASSERT_TRUE(wait_for_status_count(1, 1s));
    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_EQ(last_status_.message, "RADAR_CAMERA");
}

TEST_F(FusionNodeTest, CameraDetectionsCreateConfirmedTracksWhenFusionEnabled) {
    auto enabled_node = std::make_shared<radar::fusion::FusionNode>(
        rclcpp::NodeOptions().append_parameter_override("enable_camera_fusion", true));
    executor_.remove_node(fusion_node_);
    fusion_node_.reset();
    fusion_node_ = enabled_node;
    executor_.add_node(fusion_node_);
    ASSERT_TRUE(wait_for_discovery(true)) << "ROS entities failed to rediscover after enabling "
                                             "camera fusion";

    {
        std::lock_guard<std::mutex> lock(mutex_);
        fused_track_publish_count_     = 0;
        last_fused_track_marker_count_ = 0;
    }

    auto publish_camera = [this](double x, double y, int32_t sec, uint32_t nsec) {
        geometry_msgs::msg::PoseArray detections;
        detections.header.frame_id      = "map";
        detections.header.stamp.sec     = sec;
        detections.header.stamp.nanosec = nsec;
        geometry_msgs::msg::Pose pose;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = 0.0;
        detections.poses.push_back(pose);
        camera_detection_pub_->publish(detections);
    };

    publish_camera(1.0, 2.0, 0, 0u);
    std::this_thread::sleep_for(1000ms);
    publish_camera(1.4, 2.0, 1, 0u);
    std::this_thread::sleep_for(1000ms);
    publish_camera(1.8, 2.0, 2, 0u);

    ASSERT_TRUE(wait_for_fused_track_publish_count(3, 500ms));
    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_GT(last_fused_track_marker_count_, 0u);
}

TEST_F(FusionNodeTest, CameraDetectionNearLidarTrackKeepsFusedOutputActive) {
    auto enabled_node = std::make_shared<radar::fusion::FusionNode>(
        rclcpp::NodeOptions().append_parameter_override("enable_camera_fusion", true));
    executor_.remove_node(fusion_node_);
    fusion_node_.reset();
    fusion_node_ = enabled_node;
    executor_.add_node(fusion_node_);
    ASSERT_TRUE(wait_for_discovery(true)) << "ROS entities failed to rediscover after enabling "
                                             "camera fusion";

    cluster_pub_->publish(make_cluster_msg(0.0, 0.0, 0.0, 0, 0u));
    std::this_thread::sleep_for(1000ms);
    cluster_pub_->publish(make_cluster_msg(0.8, 0.0, 0.0, 1, 0u));
    std::this_thread::sleep_for(1000ms);
    cluster_pub_->publish(make_cluster_msg(1.6, 0.0, 0.0, 2, 0u));
    ASSERT_TRUE(wait_for_track_marker_count(3, 500ms));

    {
        std::lock_guard<std::mutex> lock(mutex_);
        fused_track_publish_count_     = 0;
        last_fused_track_marker_count_ = 0;
    }

    geometry_msgs::msg::PoseArray detections;
    detections.header.frame_id      = "map";
    detections.header.stamp.sec     = 3;
    detections.header.stamp.nanosec = 0u;
    geometry_msgs::msg::Pose pose;
    pose.position.x = 1.8;
    pose.position.y = 0.0;
    pose.position.z = 0.0;
    detections.poses.push_back(pose);
    camera_detection_pub_->publish(detections);

    ASSERT_TRUE(wait_for_fused_track_publish_count(1, 500ms));
    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_GT(last_fused_track_marker_count_, 0u);
}

TEST_F(FusionNodeTest, EmptyCameraFramesDoNotDeleteLidarTrack) {
    auto enabled_node = std::make_shared<radar::fusion::FusionNode>(
        rclcpp::NodeOptions().append_parameter_override("enable_camera_fusion", true));
    executor_.remove_node(fusion_node_);
    fusion_node_.reset();
    fusion_node_ = enabled_node;
    executor_.add_node(fusion_node_);
    ASSERT_TRUE(wait_for_discovery(true)) << "ROS entities failed to rediscover after enabling "
                                             "camera fusion";

    cluster_pub_->publish(make_cluster_msg(0.0, 0.0, 0.0, 0, 0u));
    std::this_thread::sleep_for(1000ms);
    cluster_pub_->publish(make_cluster_msg(0.8, 0.0, 0.0, 1, 0u));
    std::this_thread::sleep_for(1000ms);
    cluster_pub_->publish(make_cluster_msg(1.6, 0.0, 0.0, 2, 0u));
    ASSERT_TRUE(wait_for_track_marker_count(3, 500ms));

    {
        std::lock_guard<std::mutex> lock(mutex_);
        fused_track_publish_count_     = 0;
        last_fused_track_marker_count_ = 0;
    }

    geometry_msgs::msg::PoseArray detections;
    detections.header.frame_id      = "map";
    detections.header.stamp.sec     = 2;
    detections.header.stamp.nanosec = 100000000u;
    camera_detection_pub_->publish(detections);
    std::this_thread::sleep_for(100ms);
    detections.header.stamp.sec     = 2;
    detections.header.stamp.nanosec = 200000000u;
    camera_detection_pub_->publish(detections);

    ASSERT_TRUE(wait_for_fused_track_publish_count(2, 500ms));
    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_GT(last_fused_track_marker_count_, 0u);
}
