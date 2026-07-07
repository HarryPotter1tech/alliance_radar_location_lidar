#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cmath>
#include <filesystem>
#include <gtest/gtest.h>

#include "radar_lidar/map_data.hpp"

namespace {

auto make_test_pcd(const std::string& path, int n_points, double spacing = 0.2)
    -> const std::string& {
    auto cloud     = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    const int side = static_cast<int>(std::sqrt(static_cast<double>(n_points)));
    cloud->reserve(static_cast<size_t>(side) * side);
    for (int i = 0; i < side; ++i)
        for (int j = 0; j < side; ++j)
            cloud->emplace_back(i * spacing, j * spacing, 0.0);
    cloud->width    = cloud->size();
    cloud->height   = 1;
    cloud->is_dense = true;
    pcl::io::savePCDFileBinary(path, *cloud);
    return path;
}

class MapDataTest : public ::testing::Test {
protected:
    void SetUp() override { make_test_pcd(test_pcd_, 100, 0.2); }
    void TearDown() override { std::filesystem::remove(test_pcd_); }

    static constexpr const char* test_pcd_ = "/tmp/radar_test_map.pcd";
};

} // namespace

TEST_F(MapDataTest, LoadValidPCD) {
    auto result = radar::lidar::MapData::load(test_pcd_, 0.1);
    ASSERT_TRUE(result.has_value()) << result.error();
    auto map = *result;
    EXPECT_GT(map->size(), 0u);
}

TEST_F(MapDataTest, LoadNonexistentPCD) {
    auto result = radar::lidar::MapData::load("/tmp/nonexistent_map.pcd", 0.1);
    EXPECT_FALSE(result.has_value());
    EXPECT_NE(result.error().find("Failed to load"), std::string::npos);
}

TEST_F(MapDataTest, VoxelDownsampling) {
    auto result = radar::lidar::MapData::load(test_pcd_, 0.1);
    ASSERT_TRUE(result.has_value());
    auto map = *result;
    EXPECT_GT(map->size(), 0u);

    auto result2 = radar::lidar::MapData::load(test_pcd_, 0.5);
    ASSERT_TRUE(result2.has_value());
    auto map2 = *result2;
    EXPECT_LT(map2->size(), map->size());
}

TEST_F(MapDataTest, KdTreeAccessible) {
    auto result = radar::lidar::MapData::load(test_pcd_, 0.1);
    ASSERT_TRUE(result.has_value());
    auto map = *result;

    const auto& tree = map->sgicp_tree();
    EXPECT_EQ(tree.kdtree.indices.size(), map->size());

    const auto& pcl_tree = map->pcl_tree();
    std::vector<int> idx(1);
    std::vector<float> dist(1);
    const pcl::PointXYZ query(0.0f, 0.0f, 0.0f);
    int found = pcl_tree.nearestKSearch(query, 1, idx, dist);
    EXPECT_EQ(found, 1);
    EXPECT_EQ(idx[0], 0);
}
