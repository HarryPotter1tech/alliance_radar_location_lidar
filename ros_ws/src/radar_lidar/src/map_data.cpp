#include "radar_lidar/map_data.hpp"

#include <algorithm>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <small_gicp/ann/kdtree_omp.hpp>
#include <small_gicp/util/downsampling_omp.hpp>

#include <thread>

namespace radar::lidar {

auto MapData::load(const std::string& pcd_path, double voxel_leaf_size)
    -> std::expected<std::shared_ptr<MapData>, std::string> {
    // 1. PCL 加载
    auto raw = pcl::make_shared<PclCloud>();
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *raw) == -1) {
        return std::unexpected("Failed to load PCD: " + pcd_path);
    }

    // 2. 下采样 (PCL VoxelGrid)
    auto filtered = pcl::make_shared<PclCloud>();
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
    vg.setInputCloud(raw);
    vg.filter(*filtered);

    if (filtered->empty()) {
        return std::unexpected("Map PCD is empty after filtering");
    }

    // 3. 转 Eigen 向量 (small_gicp 构造函数自动处理齐次坐标 w=1)
    std::vector<Eigen::Vector3d> points;
    points.reserve(filtered->size());
    for (const auto& pt : filtered->points) {
        points.emplace_back(pt.x, pt.y, pt.z);
    }

    // 4. 构建 small_gicp 点云和 KD-tree
    auto sgicp_cloud         = std::make_shared<SGicpCloud>(points);
    const auto build_threads = std::max(1u, std::thread::hardware_concurrency());
    auto sgicp_tree          = std::make_shared<SGicpTree>(
        sgicp_cloud, small_gicp::KdTreeBuilderOMP(static_cast<int>(build_threads)));

    // 5. 构建 PCL KD-tree
    auto pcl_cloud = filtered;
    auto pcl_tree  = pcl::make_shared<PclTree>();
    pcl_tree->setInputCloud(pcl_cloud);

    // 6. 组装
    auto data          = std::shared_ptr<MapData>(new MapData());
    data->sgicp_cloud_ = sgicp_cloud;
    data->sgicp_tree_  = sgicp_tree;
    data->pcl_cloud_   = pcl_cloud;
    data->pcl_tree_    = pcl_tree;

    return data;
}

} // namespace radar::lidar
