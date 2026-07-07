#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <filesystem>
#include <gtest/gtest.h>

#include "radar_lidar/localization.hpp"
#include "radar_lidar/map_data.hpp"

namespace {

auto make_dense_map_cube() -> pcl::PointCloud<pcl::PointXYZ>::Ptr {
    auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    for (double x = 0.0; x <= 35.0; x += 0.5) {
        for (double y = -12.0; y <= 10.0; y += 0.5) {
            for (double z = 0.0; z <= 6.0; z += 1.0) {
                cloud->emplace_back(x, y, z);
            }
        }
    }
    cloud->width    = cloud->size();
    cloud->height   = 1;
    cloud->is_dense = true;
    return cloud;
}

class LocalizationRoiTest : public ::testing::Test {
protected:
    void SetUp() override { pcl::io::savePCDFileBinary(map_pcd_, *make_dense_map_cube()); }
    void TearDown() override { std::filesystem::remove(map_pcd_); }

    static constexpr const char* map_pcd_ = "/tmp/radar_test_localization_roi_map.pcd";
};

} // namespace

TEST_F(LocalizationRoiTest, RejectsCloudWhenOnlyOutsideTdtLidarRoiRemain) {
    auto map_result = radar::lidar::MapData::load(map_pcd_, 0.1);
    ASSERT_TRUE(map_result.has_value()) << map_result.error();

    radar::lidar::config::LocalizationConfig cfg;
    cfg.use_spherical_grid = false;
    cfg.accumulate_frames  = 0;

    radar::lidar::LocalizationStage localization(*map_result, cfg);

    radar::lidar::types::Frame frame;
    for (int i = 0; i < 80; ++i) {
        frame.points.emplace_back(4.0, -2.0 + i * 0.01, 1.0);
    }
    for (int i = 0; i < 80; ++i) {
        frame.points.emplace_back(10.0, 9.0 + i * 0.01, 1.0);
    }

    auto result = localization.process(frame);
    ASSERT_FALSE(result.has_value());
    EXPECT_NE(result.error().find("Too few points after preprocessing"), std::string::npos);
}
