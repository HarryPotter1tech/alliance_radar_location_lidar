#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <format>
#include <fstream>
#include <numbers>
#include <print>
#include <ranges>
#include <string>
#include <vector>

#include "radar_lidar/geometry_utils.hpp"
#include "radar_lidar/localization.hpp"
#include "radar_lidar/map_data.hpp"
#include "radar_lidar/offline_visualization.hpp"
#include "radar_lidar/types.hpp"

namespace {

constexpr auto deg_to_rad(double deg) -> double { return deg * std::numbers::pi / 180.0; }
constexpr auto rad_to_deg(double rad) -> double { return rad * 180.0 / std::numbers::pi; }

template <typename PointT>
auto to_rosmsg(const pcl::PointCloud<PointT>& cloud, const std::string& frame_id)
    -> sensor_msgs::msg::PointCloud2 {
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header.frame_id = frame_id;
    return msg;
}

// 候选评分: 把 source 点用 T 变到地图系, 在地图 KdTree 查最近邻,
// 统计内点率 (dist<threshold) 与内点 RMSE。归一化, 可跨起点比较。
struct RegistrationScore {
    double inlier_ratio = 0.0;
    double rmse         = std::numeric_limits<double>::max();
};

// 评分排序: 内点率优先, 内点率相近时 RMSE 次之 (数值更小更优)。
// 粗配准候选选择与精配准是否采纳共用同一套比较规则。
auto is_better_score(const RegistrationScore& a, const RegistrationScore& b) -> bool {
    if (std::abs(a.inlier_ratio - b.inlier_ratio) > 1e-6) {
        return a.inlier_ratio > b.inlier_ratio;
    }
    return a.rmse < b.rmse;
}

auto score_alignment(const pcl::KdTreeFLANN<pcl::PointXYZ>& map_tree,
    const radar::lidar::types::PointCloud& source, const Eigen::Isometry3d& T,
    double inlier_threshold) -> RegistrationScore {
    const double th2 = inlier_threshold * inlier_threshold;
    std::vector<int> idx(1);
    std::vector<float> sq_dist(1);
    size_t inliers     = 0;
    double sq_dist_sum = 0.0;
    for (const auto& p : source) {
        const Eigen::Vector3d tp = T * p;
        pcl::PointXYZ query(
            static_cast<float>(tp.x()), static_cast<float>(tp.y()), static_cast<float>(tp.z()));
        if (map_tree.nearestKSearch(query, 1, idx, sq_dist) > 0 && sq_dist[0] <= th2) {
            ++inliers;
            sq_dist_sum += sq_dist[0];
        }
    }
    RegistrationScore s;
    if (!source.empty()) {
        s.inlier_ratio = static_cast<double>(inliers) / static_cast<double>(source.size());
    }
    if (inliers > 0) {
        s.rmse = std::sqrt(sq_dist_sum / static_cast<double>(inliers));
    }
    return s;
}

// 输出外参 T_map_lidar (工作系: 中心原点/米/Z-up) + 评分, 供后续标定/上报使用。
auto write_pose_json(const std::string& path, const Eigen::Isometry3d& T,
    const RegistrationScore& score, bool converged) -> std::expected<void, std::string> {
    std::ofstream ofs(path);
    if (!ofs) {
        return std::unexpected("cannot open " + path);
    }
    const Eigen::Vector3d t = T.translation();
    const Eigen::Quaterniond q(T.rotation());
    const Eigen::Matrix4d m = T.matrix();
    ofs << std::format("{{\n");
    ofs << std::format("  \"frame\": \"work_center_origin_zup_meters\",\n");
    ofs << std::format("  \"direction\": \"T_map_lidar (lidar point -> map)\",\n");
    ofs << std::format("  \"translation\": [{:.6f}, {:.6f}, {:.6f}],\n", t.x(), t.y(), t.z());
    ofs << std::format(
        "  \"quaternion_xyzw\": [{:.6f}, {:.6f}, {:.6f}, {:.6f}],\n", q.x(), q.y(), q.z(), q.w());
    ofs << std::format("  \"matrix\": [\n");
    for (int r = 0; r < 4; ++r) {
        ofs << std::format("    [{:.6f}, {:.6f}, {:.6f}, {:.6f}]{}\n", m(r, 0), m(r, 1), m(r, 2),
            m(r, 3), r < 3 ? "," : "");
    }
    ofs << std::format("  ],\n");
    ofs << std::format("  \"inlier_ratio\": {:.6f},\n", score.inlier_ratio);
    ofs << std::format("  \"rmse\": {:.6f},\n", score.rmse);
    ofs << std::format("  \"converged\": {}\n", converged);
    ofs << std::format("}}\n");
    return { };
}

} // namespace

class OfflineTestNode : public rclcpp::Node {
public:
    OfflineTestNode()
        : Node("offline_test_node",
              rclcpp::NodeOptions { }.automatically_declare_parameters_from_overrides(true))
        , map_(nullptr)
        , localization_(nullptr, { }) {
        init();
    }

private:
    void init() {
        std::string map_path;
        std::string scan_path;
        get_parameter("output_frame", output_frame_);
        get_parameter("scan_frame", scan_frame_);
        get_parameter("map_path", map_path);
        get_parameter("scan_path", scan_path);

        if (map_path.empty() || scan_path.empty()) {
            std::println(stderr, "ERROR: map_path and scan_path required");
            rclcpp::shutdown();
            return;
        }

        const auto qos = rclcpp::QoS(1).transient_local();
        pub_map_       = create_publisher<sensor_msgs::msg::PointCloud2>("/offline/map", qos);
        pub_raw_       = create_publisher<sensor_msgs::msg::PointCloud2>("/offline/scan_raw", qos);
        pub_roi_       = create_publisher<sensor_msgs::msg::PointCloud2>("/offline/scan_roi", qos);
        pub_aligned_ =
            create_publisher<sensor_msgs::msg::PointCloud2>("/offline/scan_aligned", qos);
        pub_overlay_ = create_publisher<sensor_msgs::msg::PointCloud2>("/offline/overlay", qos);
        pub_pose_ =
            create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/offline/pose", qos);
        pub_diag_ =
            create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("/offline/diagnostics", qos);

        double voxel_leaf = 0.1, scan_voxel = 0.5;
        bool map_y_up = false;
        get_parameter("voxel_leaf", voxel_leaf);
        get_parameter("scan_voxel", scan_voxel);
        get_parameter("map_y_up", map_y_up);
        if (map_y_up) {
            // 兼容回退（已弃用）：推荐改用 model_to_map --y-up 从 FBX 直接生成 Z-up 地图。
            // 该分支每次运行都重转+落临时文件，仅为兼容旧的 Y-up 地图保留。
            std::println(stderr,
                "[offline] WARNING: map_y_up=true 已弃用，请改用 model_to_map --y-up 生成 Z-up "
                "地图。");
            auto raw = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
            if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_path, *raw) == -1) {
                std::println(stderr, "ERROR: Failed to load map PCD for rotation");
                rclcpp::shutdown();
                return;
            }
            for (auto& pt : raw->points) {
                const float y = pt.y, z = pt.z;
                pt.y = -z;
                pt.z = y;
            }
            map_path = "/tmp/map_z_up.pcd";
            pcl::io::savePCDFileBinary(map_path, *raw);
            std::println("[offline] Map rotated Y-up→Z-up (deprecated), saved to {}", map_path);
        }
        auto map_result = radar::lidar::MapData::load(map_path, voxel_leaf);
        if (!map_result) {
            std::println(stderr, "ERROR: Map load failed: {}", map_result.error());
            rclcpp::shutdown();
            return;
        }
        map_ = *map_result;
        std::println("[offline] Map: {} points", map_->size());
        const auto colored_map = radar::lidar::offline::make_colored_cloud(
            map_->pcl_cloud(), radar::lidar::offline::kMapColorBgr);
        pub_map_->publish(to_rosmsg(colored_map, output_frame_));

        auto scan_pcl = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(scan_path, *scan_pcl) == -1) {
            std::println(stderr, "ERROR: Failed to load scan: {}", scan_path);
            rclcpp::shutdown();
            return;
        }
        if (scan_voxel > 0.0) {
            auto downsampled = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
            pcl::VoxelGrid<pcl::PointXYZ> vg;
            vg.setLeafSize(scan_voxel, scan_voxel, scan_voxel);
            vg.setInputCloud(scan_pcl);
            vg.filter(*downsampled);
            std::println("[offline] Scan downsampled: {} → {} points (voxel={})", scan_pcl->size(),
                downsampled->size(), scan_voxel);
            scan_pcl = downsampled;
        }
        auto frame = radar::lidar::geom::filter_valid_points(*scan_pcl);
        std::println("[offline] Scan: {} valid points", frame.points.size());
        if (frame.points.size() < 100) {
            std::println(stderr, "ERROR: Too few points");
            rclcpp::shutdown();
            return;
        }
        const auto colored_raw = radar::lidar::offline::make_colored_cloud(
            frame.points, radar::lidar::offline::kRawScanColorBgr);
        pub_raw_->publish(to_rosmsg(colored_raw, scan_frame_));

        // 先读初始位姿——后续 ROI 扇形和 GICP 都依赖它
        double init_x = 0.0, init_y = 0.0, init_z = 0.0, init_yaw_deg = 0.0;
        get_parameter("initial_x", init_x);
        get_parameter("initial_y", init_y);
        get_parameter("initial_z", init_z);
        get_parameter("initial_yaw_deg", init_yaw_deg);

        // Sector ROI: fan-shaped filter centered at radar position in scan frame
        double roi_half_fov_deg = 60.0, roi_min_range = 1.0, roi_max_range = 30.0;
        double roi_z_min = 0.0, roi_z_max = 7.0;
        double roi_origin_x = 0.0, roi_origin_y = 0.0, roi_origin_yaw_deg = 0.0;
        bool use_roi = false;
        get_parameter("use_roi", use_roi);
        get_parameter("roi_half_fov_deg", roi_half_fov_deg);
        get_parameter("roi_min_range", roi_min_range);
        get_parameter("roi_max_range", roi_max_range);
        get_parameter("roi_z_min", roi_z_min);
        get_parameter("roi_z_max", roi_z_max);
        get_parameter("roi_origin_x", roi_origin_x);
        get_parameter("roi_origin_y", roi_origin_y);
        get_parameter("roi_origin_yaw_deg", roi_origin_yaw_deg);
        if (use_roi) {
            const double yaw_rad   = deg_to_rad(roi_origin_yaw_deg);
            const double cos_yaw   = std::cos(yaw_rad);
            const double sin_yaw   = std::sin(yaw_rad);
            const double half_fov  = deg_to_rad(roi_half_fov_deg);
            const double tan2_half = std::tan(half_fov) * std::tan(half_fov);
            const double min_r2    = roi_min_range * roi_min_range;
            const double max_r2    = roi_max_range * roi_max_range;
            auto clipped           = radar::lidar::types::PointCloud();
            clipped.reserve(frame.points.size());
            for (const auto& p : frame.points) {
                if (p.z() < roi_z_min || p.z() > roi_z_max) continue;
                const double dx = p.x() - roi_origin_x;
                const double dy = p.y() - roi_origin_y;
                const double r2 = dx * dx + dy * dy;
                if (r2 < min_r2 || r2 > max_r2) continue;
                const double fx = dx * cos_yaw + dy * sin_yaw;
                const double fy = -dx * sin_yaw + dy * cos_yaw;
                if (fx <= 0.0) continue;
                if (fy * fy > fx * fx * tan2_half) continue;
                clipped.push_back(p);
            }
            std::println("[offline] ROI sector: {} → {} points (origin=({:.1f},{:.1f}) yaw={}° "
                         "FOV=±{}°)",
                frame.points.size(), clipped.size(), roi_origin_x, roi_origin_y, roi_origin_yaw_deg,
                roi_half_fov_deg);
            if (clipped.size() < 100) {
                std::println(stderr, "ERROR: Too few points after ROI");
                rclcpp::shutdown();
                return;
            }
            const auto colored_roi = radar::lidar::offline::make_colored_cloud(
                clipped, radar::lidar::offline::kRoiColorBgr);
            pub_roi_->publish(to_rosmsg(colored_roi, scan_frame_));
            frame.points = std::move(clipped);
        }

        radar::lidar::config::LocalizationConfig cfg;
        cfg.voxel_leaf_size = voxel_leaf;
        get_parameter("max_corr_distance", cfg.max_corr_distance);
        get_parameter("max_iterations", cfg.max_iterations);
        get_parameter("num_threads", cfg.num_threads);
        get_parameter("use_spherical_grid", cfg.use_spherical_grid);
        get_parameter("spherical_grid_deg", cfg.spherical_grid_deg);
        get_parameter("accumulate_frames", cfg.accumulate_frames);
        get_parameter("verbose", cfg.verbose);

        // look-at 初值参数
        bool use_look_at = true;
        double look_at_x = 0.0, look_at_y = 0.0, look_at_z = 0.5;
        double init_pitch_deg = 0.0;
        get_parameter("use_look_at", use_look_at);
        get_parameter("look_at_x", look_at_x);
        get_parameter("look_at_y", look_at_y);
        get_parameter("look_at_z", look_at_z);
        get_parameter("initial_pitch_deg", init_pitch_deg);

        // yaw 局部多起点搜索参数
        double yaw_search_range_deg = 30.0, yaw_search_step_deg = 10.0;
        double inlier_threshold = 0.3;
        get_parameter("yaw_search_range_deg", yaw_search_range_deg);
        get_parameter("yaw_search_step_deg", yaw_search_step_deg);
        get_parameter("inlier_threshold", inlier_threshold);

        const Eigen::Vector3d eye(init_x, init_y, init_z);
        double base_yaw   = deg_to_rad(init_yaw_deg);
        double base_pitch = deg_to_rad(init_pitch_deg);
        if (use_look_at) {
            const auto [yaw, pitch] =
                radar::lidar::geom::look_at_yaw_pitch(eye, { look_at_x, look_at_y, look_at_z });
            base_yaw   = yaw;
            base_pitch = pitch;
        }
        std::println("[offline] Init pose: eye=({:.2f},{:.2f},{:.2f}) yaw={:.2f}° pitch={:.2f}° "
                     "(look_at={})",
            init_x, init_y, init_z, rad_to_deg(base_yaw), rad_to_deg(base_pitch), use_look_at);

        // 粗配准: yaw 多起点搜索时每个候选跑一次 (大 voxel/大 corr/少迭代)
        auto coarse_cfg              = cfg;
        coarse_cfg.voxel_leaf_size   = 0.5;
        coarse_cfg.max_corr_distance = 30.0;
        coarse_cfg.max_iterations    = 50;
        coarse_cfg.roi.use_roi       = false;
        get_parameter("coarse_voxel", coarse_cfg.voxel_leaf_size);
        get_parameter("coarse_max_corr", coarse_cfg.max_corr_distance);
        get_parameter("coarse_max_iter", coarse_cfg.max_iterations);

        std::vector<double> yaw_offsets;
        if (yaw_search_step_deg > 0.0 && yaw_search_range_deg > 0.0) {
            for (double off = -yaw_search_range_deg; off <= yaw_search_range_deg + 1e-9;
                off += yaw_search_step_deg) {
                yaw_offsets.push_back(deg_to_rad(off));
            }
        } else {
            yaw_offsets.push_back(0.0);
        }

        // 每个 yaw 候选: 从对应初值跑粗配准, 用地图 KdTree 归一化评分
        struct Candidate {
            Eigen::Isometry3d t_map_lidar;
            RegistrationScore score;
            double yaw_offset_deg;
            bool converged;
        };
        std::vector<Candidate> candidates;
        candidates.reserve(yaw_offsets.size());
        for (const double yaw_off : yaw_offsets) {
            auto init_pose =
                radar::lidar::geom::pose_from_yaw_pitch(eye, base_yaw + yaw_off, base_pitch);
            auto coarse_stage = radar::lidar::LocalizationStage(map_, coarse_cfg);
            coarse_stage.set_initial_pose(init_pose);
            auto coarse_result             = coarse_stage.process(frame);
            const Eigen::Isometry3d cand_T = coarse_result ? coarse_result->t_map_lidar : init_pose;
            const auto score =
                score_alignment(map_->pcl_tree(), frame.points, cand_T, inlier_threshold);
            candidates.push_back({ cand_T, score, rad_to_deg(yaw_off),
                coarse_result ? coarse_result->converged : false });
            std::println("[offline]   yaw_off={:+.1f}° → inlier={:.3f} rmse={:.4f}",
                rad_to_deg(yaw_off), score.inlier_ratio, score.rmse);
        }

        const auto best = std::ranges::max_element(candidates,
            [](const auto& a, const auto& b) { return is_better_score(b.score, a.score); });
        std::println("[offline] Best coarse: yaw_off={:+.1f}° inlier={:.3f} rmse={:.4f}",
            best->yaw_offset_deg, best->score.inlier_ratio, best->score.rmse);

        // 精配准: 从最优候选出发收敛
        auto fine_cfg              = cfg;
        fine_cfg.max_corr_distance = 3.0;
        fine_cfg.roi.use_roi       = false;
        get_parameter("fine_max_corr", fine_cfg.max_corr_distance);
        auto fine_stage = radar::lidar::LocalizationStage(map_, fine_cfg);
        fine_stage.set_initial_pose(best->t_map_lidar);
        auto fine_result = fine_stage.process(frame);

        Eigen::Isometry3d t_map_lidar   = best->t_map_lidar;
        bool converged                  = best->converged;
        Eigen::Matrix<double, 6, 6> cov = Eigen::Matrix<double, 6, 6>::Identity();
        RegistrationScore final_score   = best->score;
        if (fine_result) {
            const auto fine_sc = score_alignment(
                map_->pcl_tree(), frame.points, fine_result->t_map_lidar, inlier_threshold);
            // 精配准结果更优才采纳, 否则保留粗配准 (防止精配准把好起点带偏)
            if (is_better_score(fine_sc, best->score)) {
                t_map_lidar = fine_result->t_map_lidar;
                converged   = fine_result->converged;
                cov         = fine_result->covariance;
                final_score = fine_sc;
                std::println("[offline] Fine OK. inlier={:.3f} rmse={:.4f}", fine_sc.inlier_ratio,
                    fine_sc.rmse);
            } else {
                std::println("[offline] Fine worse (inlier={:.3f} < coarse={:.3f}), keeping coarse",
                    fine_sc.inlier_ratio, best->score.inlier_ratio);
            }
        } else {
            std::println(stderr, "[offline] Fine registration failed, keeping coarse");
        }
        const double fitness_score = final_score.rmse;

        // T_target_source maps scan → map; 把 scan 变换到地图系, 使 topic 名字
        // (scan_aligned) 与内容语义一致, 且和地图共处同一 frame (output_frame_)。
        auto scan_in_map = frame.points | std::views::filter([](const auto& pt) {
            return radar::lidar::offline::is_valid_xyz(pt);
        }) | std::views::transform([&t_map_lidar](const auto& pt) { return t_map_lidar * pt; })
            | std::ranges::to<radar::lidar::types::PointCloud>();
        const auto colored_aligned = radar::lidar::offline::make_colored_cloud(
            scan_in_map, radar::lidar::offline::kAlignedScanColorBgr);
        pub_aligned_->publish(to_rosmsg(colored_aligned, output_frame_));

        // Overlay: 地图 (固定, 青色) + scan 对齐结果 (绿色), 都在地图系
        const auto colored_overlay =
            radar::lidar::offline::make_overlay_cloud(map_->pcl_cloud(), scan_in_map,
                radar::lidar::offline::kMapColorBgr, radar::lidar::offline::kAlignedScanColorBgr);
        pub_overlay_->publish(to_rosmsg(colored_overlay, output_frame_));

        const auto trans = t_map_lidar.translation();
        const Eigen::Quaterniond q(t_map_lidar.rotation());
        geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.stamp            = now();
        pose_msg.header.frame_id         = output_frame_;
        pose_msg.pose.pose.position.x    = trans.x();
        pose_msg.pose.pose.position.y    = trans.y();
        pose_msg.pose.pose.position.z    = trans.z();
        pose_msg.pose.pose.orientation.x = q.x();
        pose_msg.pose.pose.orientation.y = q.y();
        pose_msg.pose.pose.orientation.z = q.z();
        pose_msg.pose.pose.orientation.w = q.w();
        Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(pose_msg.pose.covariance.data()) =
            cov;
        pub_pose_->publish(pose_msg);

        diagnostic_msgs::msg::DiagnosticStatus diag;
        diag.name    = "offline_registration";
        diag.level   = converged ? diagnostic_msgs::msg::DiagnosticStatus::OK
                                 : diagnostic_msgs::msg::DiagnosticStatus::WARN;
        diag.message = std::format("inlier={:.3f} rmse={:.4f} converged={}",
            final_score.inlier_ratio, final_score.rmse, converged);
        pub_diag_->publish(diag);

        std::string pose_out;
        get_parameter("pose_out", pose_out);
        if (!pose_out.empty()) {
            if (auto r = write_pose_json(pose_out, t_map_lidar, final_score, converged); !r) {
                std::println(stderr, "[offline] ERROR writing pose json: {}", r.error());
            } else {
                std::println("[offline] Pose written: {}", pose_out);
            }
        }

        std::println("[offline] Result: inlier={:.3f} rmse={:.4f} pos=({:.3f},{:.3f},{:.3f})",
            final_score.inlier_ratio, final_score.rmse, trans.x(), trans.y(), trans.z());
        std::println("[offline] All topics published. Spinning.");
    }

    std::shared_ptr<const radar::lidar::MapData> map_;
    radar::lidar::LocalizationStage localization_;
    std::string output_frame_ { "map" };
    std::string scan_frame_ { "scan" };

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_, pub_raw_, pub_roi_,
        pub_aligned_, pub_overlay_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr pub_diag_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OfflineTestNode>());
    rclcpp::shutdown();
    return 0;
}
