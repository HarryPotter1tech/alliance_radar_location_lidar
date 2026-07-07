#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace radar::lidar::types {

using Point      = Eigen::Vector3d;
using PointCloud = std::vector<Point>;
using Timestamp  = int64_t;

struct Frame {
    PointCloud points;
    Timestamp stamp { 0 };
    std::string frame_id;
};

struct PoseEstimate {
    Eigen::Isometry3d t_map_lidar          = Eigen::Isometry3d::Identity();
    Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Identity();
    double fitness_score                   = 0.0;
    bool converged                         = false;
};

} // namespace radar::lidar::types
