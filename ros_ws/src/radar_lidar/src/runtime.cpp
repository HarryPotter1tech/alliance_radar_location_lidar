#include <rclcpp/rclcpp.hpp>

#include "radar_lidar/pipeline.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<radar::lidar::LidarPipeline>();
    if (!rclcpp::ok()) {
        // 构造函数已因 map_path 缺失/地图加载失败调用 rclcpp::shutdown()
        return 1;
    }
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
