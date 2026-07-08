#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <charconv>
#include <cmath>
#include <expected>
#include <format>
#include <fstream>
#include <iomanip>
#include <numbers>
#include <print>
#include <ranges>
#include <string>
#include <string_view>
#include <vector>

#include "radar_lidar/geometry_utils.hpp"
#include "radar_lidar/localization.hpp"
#include "radar_lidar/map_data.hpp"
#include "radar_lidar/types.hpp"

namespace {

constexpr auto deg_to_rad(double deg) -> double { return deg * std::numbers::pi / 180.0; }

constexpr auto rad_to_deg(double rad) -> double { return rad * 180.0 / std::numbers::pi; }

struct Args {
    std::string map_path;
    std::string scan_path;
    double voxel_leaf      = 0.1;
    double scan_voxel      = 0.1;
    double max_corr        = 1.0;
    int max_iter           = 48;
    int num_threads        = 4;
    double init_x          = 0.0;
    double init_y          = 0.0;
    double init_z          = 0.0;
    double init_yaw_deg    = 0.0;
    std::string output_pcd = "aligned.pcd";
    std::string pose_out   = "pose.json";
    bool verbose           = false;
};

auto usage(std::string_view prog) -> std::string {
    return std::format("Usage: {} <map.pcd> <scan.pcd> [options]\n"
                       "Options:\n"
                       "  --voxel <float>       map voxel size (default 0.1)\n"
                       "  --scan-voxel <float>   scan VoxelGrid downsampling (default 0.1, 0=off)\n"
                       "  --max-corr <float>    GICP max correspondence distance (default 1.0)\n"
                       "  --max-iter <int>      GICP max iterations (default 48)\n"
                       "  --num-threads <int>   parallel threads (default 4)\n"
                       "  --init-x/y/z <float>  initial pose translation (default 0)\n"
                       "  --init-yaw <float>    initial pose yaw in degrees (default 0)\n"
                       "  --output <path>       aligned PCD output (default aligned.pcd)\n"
                       "  --pose-out <path>     pose JSON output (default pose.json)\n"
                       "  --verbose             verbose GICP output\n",
        prog);
}

template <typename T> auto parse_number(std::string_view sv) -> std::expected<T, std::string> {
    T value { };
    const auto [ptr, ec] = std::from_chars(sv.data(), sv.data() + sv.size(), value);
    if (ec != std::errc { } || ptr != sv.data() + sv.size()) {
        return std::unexpected(std::format("Invalid number: '{}'", sv));
    }
    return value;
}

template <typename T>
auto checked_assign(T& dest, std::string_view val) -> std::expected<void, std::string> {
    auto n = parse_number<T>(val);
    if (!n) return std::unexpected(n.error());
    dest = *n;
    return { };
}

// 校验数值参数完全消费字符串, 且满足下界要求 (min_exclusive=true 时严格 > min)。
template <typename T>
auto checked_assign_bounded(T& dest, std::string_view val, std::string_view name, T min,
    bool min_exclusive) -> std::expected<void, std::string> {
    auto n = parse_number<T>(val);
    if (!n) return std::unexpected(n.error());
    if (min_exclusive ? (*n <= min) : (*n < min)) {
        return std::unexpected(
            std::format("{} must be {} {}, got '{}'", name, min_exclusive ? ">" : ">=", min, val));
    }
    dest = *n;
    return { };
}

auto parse_args(int argc, char** argv) -> std::expected<Args, std::string> {
    if (argc < 3) {
        return std::unexpected(usage(argv[0]));
    }

    Args args;
    args.map_path  = argv[1];
    args.scan_path = argv[2];

    for (int i = 3; i < argc; ++i) {
        std::string_view arg = argv[i];

        if (arg == "--verbose") {
            args.verbose = true;
            continue;
        }

        if (i + 1 >= argc) {
            return std::unexpected(
                std::format("ERROR: {} requires a value\n{}", arg, usage(argv[0])));
        }
        const std::string val = argv[++i];

        if (arg == "--output") {
            args.output_pcd = val;
            continue;
        }
        if (arg == "--pose-out") {
            args.pose_out = val;
            continue;
        }

        if (arg == "--voxel") {
            if (auto r = checked_assign_bounded(args.voxel_leaf, val, "--voxel", 0.0, true); !r)
                return std::unexpected(r.error());
        } else if (arg == "--scan-voxel") {
            if (auto r = checked_assign_bounded(args.scan_voxel, val, "--scan-voxel", 0.0, false);
                !r)
                return std::unexpected(r.error());
        } else if (arg == "--max-corr") {
            if (auto r = checked_assign_bounded(args.max_corr, val, "--max-corr", 0.0, true); !r)
                return std::unexpected(r.error());
        } else if (arg == "--max-iter") {
            if (auto r = checked_assign_bounded(args.max_iter, val, "--max-iter", 0, true); !r)
                return std::unexpected(r.error());
        } else if (arg == "--num-threads") {
            if (auto r = checked_assign_bounded(args.num_threads, val, "--num-threads", 0, true);
                !r)
                return std::unexpected(r.error());
        } else if (arg == "--init-x") {
            if (auto r = checked_assign(args.init_x, val); !r) return std::unexpected(r.error());
        } else if (arg == "--init-y") {
            if (auto r = checked_assign(args.init_y, val); !r) return std::unexpected(r.error());
        } else if (arg == "--init-z") {
            if (auto r = checked_assign(args.init_z, val); !r) return std::unexpected(r.error());
        } else if (arg == "--init-yaw") {
            if (auto r = checked_assign(args.init_yaw_deg, val); !r)
                return std::unexpected(r.error());
        } else {
            return std::unexpected(
                std::format("ERROR: unknown argument '{}'\n{}", arg, usage(argv[0])));
        }
    }
    return args;
}

auto write_pose_json(const std::string& path, const radar::lidar::types::PoseEstimate& pose)
    -> std::expected<void, std::string> {
    const auto& t_map_lidar = pose.t_map_lidar;
    const auto trans        = t_map_lidar.translation();
    const Eigen::Quaterniond q(t_map_lidar.rotation());
    const auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);

    auto fmt_row = [&](int i) {
        const auto r = pose.covariance.row(i);
        return std::format("    [{:.8f}, {:.8f}, {:.8f}, {:.8f}, {:.8f}, {:.8f}]{}", r(0), r(1),
            r(2), r(3), r(4), r(5), i < 5 ? "," : "");
    };
    auto cov = std::views::iota(0, 6) | std::views::transform(fmt_row) | std::views::join_with('\n')
        | std::ranges::to<std::string>();

    auto json = std::format("{{\n"
                            "  \"frame\": \"work_center_origin_zup_meters\",\n"
                            "  \"direction\": \"T_map_lidar (lidar point -> map)\",\n"
                            "  \"converged\": {},\n"
                            "  \"fitness_score\": {:.8f},\n"
                            "  \"translation\": [{:.8f}, {:.8f}, {:.8f}],\n"
                            "  \"rotation_quaternion\": [{:.8f}, {:.8f}, {:.8f}, {:.8f}],\n"
                            "  \"rotation_euler_xyz_deg\": [{:.8f}, {:.8f}, {:.8f}],\n"
                            "  \"covariance\": [\n"
                            "{}\n"
                            "  ]\n"
                            "}}\n",
        pose.converged ? "true" : "false", pose.fitness_score, trans.x(), trans.y(), trans.z(),
        q.x(), q.y(), q.z(), q.w(), rad_to_deg(euler.x()), rad_to_deg(euler.y()),
        rad_to_deg(euler.z()), cov);

    std::ofstream f(path);
    if (!f) {
        return std::unexpected(std::format("Cannot open file: {}", path));
    }
    f << json;
    return { };
}

auto build_merged_pcd(const pcl::PointCloud<pcl::PointXYZ>& map_cloud,
    const radar::lidar::types::PointCloud& scan, const Eigen::Isometry3d& T)
    -> pcl::PointCloud<pcl::PointXYZI>::Ptr {
    auto merged = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    merged->reserve(map_cloud.size() + scan.size());

    for (const auto& pt : map_cloud.points)
        merged->emplace_back(pt.x, pt.y, pt.z, 0.0f);
    for (const auto& pt : scan) {
        const Eigen::Vector3d tp = T * pt;
        merged->emplace_back(tp.x(), tp.y(), tp.z(), 1.0f);
    }
    merged->width    = merged->size();
    merged->height   = 1;
    merged->is_dense = true;
    return merged;
}

} // namespace

int main(int argc, char** argv) {
    auto args_result = parse_args(argc, argv);
    if (!args_result) {
        std::println(stderr, "{}", args_result.error());
        return 1;
    }
    const auto& args = *args_result;

    std::println("[registration_tool] Loading map: {}", args.map_path);
    auto map_result = radar::lidar::MapData::load(args.map_path, args.voxel_leaf);
    if (!map_result) {
        std::println(stderr, "[registration_tool] ERROR: {}", map_result.error());
        return 1;
    }
    const auto& map = *map_result;
    std::println("[registration_tool] Map loaded: {} points", map->size());

    std::println("[registration_tool] Loading scan: {}", args.scan_path);
    auto scan_pcl = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(args.scan_path, *scan_pcl) == -1) {
        std::println(stderr, "[registration_tool] ERROR: Failed to load scan PCD");
        return 1;
    }

    if (args.scan_voxel > 0.0) {
        auto downsampled = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setLeafSize(args.scan_voxel, args.scan_voxel, args.scan_voxel);
        vg.setInputCloud(scan_pcl);
        vg.filter(*downsampled);
        std::println("[registration_tool] Scan downsampled: {} → {} points (voxel={})",
            scan_pcl->size(), downsampled->size(), args.scan_voxel);
        scan_pcl = downsampled;
    }

    auto frame = radar::lidar::geom::filter_valid_points(*scan_pcl);
    std::println("[registration_tool] Scan loaded: {} valid points", frame.points.size());
    if (frame.points.size() < 100) {
        std::println(
            stderr, "[registration_tool] ERROR: Too few scan points: {}", frame.points.size());
        return 1;
    }

    radar::lidar::config::LocalizationConfig cfg;
    cfg.voxel_leaf_size   = args.voxel_leaf;
    cfg.max_corr_distance = args.max_corr;
    cfg.max_iterations    = args.max_iter;
    cfg.num_threads       = args.num_threads;
    cfg.verbose           = args.verbose;

    auto localization = radar::lidar::LocalizationStage(map, cfg);

    if (args.init_x != 0.0 || args.init_y != 0.0 || args.init_z != 0.0
        || args.init_yaw_deg != 0.0) {
        const auto init_pose = radar::lidar::geom::pose_from_yaw_pitch(
            Eigen::Vector3d(args.init_x, args.init_y, args.init_z), deg_to_rad(args.init_yaw_deg),
            0.0);
        localization.set_initial_pose(init_pose);
        std::println("[registration_tool] Initial pose: x={} y={} z={} yaw={}deg", args.init_x,
            args.init_y, args.init_z, args.init_yaw_deg);
    }

    std::println("[registration_tool] Running GICP...");
    auto result = localization.process(frame);
    if (!result) {
        std::println(stderr, "[registration_tool] GICP FAILED: {}", result.error());
        return 2;
    }

    const auto& pose        = *result;
    const auto& t_map_lidar = pose.t_map_lidar;
    const auto trans        = t_map_lidar.translation();
    const Eigen::Quaterniond q(t_map_lidar.rotation());

    std::println("[registration_tool] === Result ===");
    std::println("  converged:      {}", pose.converged ? "true" : "false");
    std::println("  fitness_score:  {}", pose.fitness_score);
    std::println("  translation:    [{}, {}, {}]", trans.x(), trans.y(), trans.z());
    std::println("  quaternion:     [{}, {}, {}, {}]", q.x(), q.y(), q.z(), q.w());

    if (auto r = write_pose_json(args.pose_out, pose); !r) {
        std::println(stderr, "[registration_tool] ERROR: {}", r.error());
        return 1;
    }
    std::println("[registration_tool] Pose written: {}", args.pose_out);

    auto merged = build_merged_pcd(map->pcl_cloud(), frame.points, t_map_lidar);
    if (pcl::io::savePCDFileBinary(args.output_pcd, *merged) != 0) {
        std::println(stderr, "[registration_tool] ERROR: Failed to write aligned PCD");
        return 1;
    }
    std::println("[registration_tool] Aligned PCD: {} ({} points, intensity 0=map 1=scan)",
        args.output_pcd, merged->size());

    return 0;
}
