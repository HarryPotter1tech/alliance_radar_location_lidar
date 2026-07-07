#include "radar_lidar/cluster.hpp"

#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <limits>

namespace radar::lidar {

ClusterStage::ClusterStage(ClusterConfig cfg)
    : cfg_(std::move(cfg)) { }

auto ClusterStage::process(const types::PointCloud& dynamic_points)
    -> std::expected<std::vector<ClusterResult>, std::string> {
    if (dynamic_points.empty()) {
        return std::vector<ClusterResult> { };
    }

    // 转 PCL 点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->reserve(dynamic_points.size());
    for (const auto& p : dynamic_points) {
        cloud->emplace_back(
            static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z()));
    }
    cloud->width    = cloud->size();
    cloud->height   = 1;
    cloud->is_dense = true;

    // 欧氏聚类
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cfg_.cluster_tolerance);
    ec.setMinClusterSize(cfg_.min_cluster_size);
    ec.setMaxClusterSize(cfg_.max_cluster_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    ec.extract(cluster_indices);

    // 计算每个聚类的质心 + AABB
    std::vector<ClusterResult> results;
    results.reserve(cluster_indices.size());

    for (const auto& indices : cluster_indices) {
        ClusterResult r;
        r.point_count = static_cast<int>(indices.indices.size());

        double sum_x = 0, sum_y = 0, sum_z = 0;
        float min_x = std::numeric_limits<float>::max();
        float min_y = std::numeric_limits<float>::max();
        float min_z = std::numeric_limits<float>::max();
        float max_x = std::numeric_limits<float>::lowest();
        float max_y = std::numeric_limits<float>::lowest();
        float max_z = std::numeric_limits<float>::lowest();

        for (int idx : indices.indices) {
            const auto& pt = cloud->points[idx];
            sum_x += pt.x;
            sum_y += pt.y;
            sum_z += pt.z;
            if (pt.x < min_x) min_x = pt.x;
            if (pt.y < min_y) min_y = pt.y;
            if (pt.z < min_z) min_z = pt.z;
            if (pt.x > max_x) max_x = pt.x;
            if (pt.y > max_y) max_y = pt.y;
            if (pt.z > max_z) max_z = pt.z;
        }

        r.centroid =
            Eigen::Vector3d(sum_x / r.point_count, sum_y / r.point_count, sum_z / r.point_count);
        r.min_bound = Eigen::Vector3d(min_x, min_y, min_z);
        r.max_bound = Eigen::Vector3d(max_x, max_y, max_z);

        results.push_back(r);
    }

    return results;
}

} // namespace radar::lidar
