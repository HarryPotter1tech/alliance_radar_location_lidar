#pragma once

#include <expected>
#include <memory>
#include <string>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <small_gicp/ann/kdtree.hpp>
#include <small_gicp/points/point_cloud.hpp>

namespace radar::lidar {

class MapData {
public:
    using SGicpCloud = small_gicp::PointCloud;
    using SGicpTree  = small_gicp::KdTree<SGicpCloud>;
    using PclCloud   = pcl::PointCloud<pcl::PointXYZ>;
    using PclTree    = pcl::KdTreeFLANN<pcl::PointXYZ>;

    static auto load(const std::string& pcd_path, double voxel_leaf_size = 0.1)
        -> std::expected<std::shared_ptr<MapData>, std::string>;

    [[nodiscard]] auto sgicp_cloud() const -> const SGicpCloud& { return *sgicp_cloud_; }
    [[nodiscard]] auto sgicp_tree() const -> const SGicpTree& { return *sgicp_tree_; }
    [[nodiscard]] auto pcl_cloud() const -> const PclCloud& { return *pcl_cloud_; }
    [[nodiscard]] auto pcl_tree() const -> const PclTree& { return *pcl_tree_; }
    [[nodiscard]] auto size() const -> size_t { return sgicp_cloud_->size(); }

private:
    MapData() = default;
    std::shared_ptr<SGicpCloud> sgicp_cloud_;
    std::shared_ptr<SGicpTree> sgicp_tree_;
    std::shared_ptr<PclCloud> pcl_cloud_;
    std::shared_ptr<PclTree> pcl_tree_;
};

} // namespace radar::lidar
