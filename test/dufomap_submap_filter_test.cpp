#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "sonar_localization/dufomap_submap_filter.hpp"

namespace fs = std::filesystem;

std::string save_pcd_dir;

namespace {

int failures = 0;

// Wall sampling step = half the dufomap resolution (defaultConfig().resolution
// is 0.2), so every voxel along the wall is guaranteed to receive a hit.
constexpr float kWallStep = 0.1f;

void check(bool ok, const std::string& msg)
{
    if (ok) {
        std::cout << "  PASS: " << msg << '\n';
    } else {
        std::cout << "  FAIL: " << msg << '\n';
        ++failures;
    }
}

dufomap_submap_filter::Config defaultConfig()
{
    dufomap_submap_filter::Config cfg;
    cfg.resolution = 0.2;
    cfg.d_s = 0.1;
    cfg.d_p = 1;
    cfg.num_threads = 1;
    cfg.hit_extension = true;
    cfg.ray_passthrough_hits = false;
    cfg.use_cluster = false;
    cfg.propagate = true;
    return cfg;
}

Eigen::Matrix4f identityPose()
{
    return Eigen::Matrix4f::Identity();
}

// Perpendicular wall in the sensor frame: constant x, points spanning y-z.
pcl::PointCloud<pcl::PointXYZ>::Ptr wallCloud(
    float wall_x,
    float half_extent = 5.0f,
    float step = kWallStep)
{
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    for (float y = -half_extent; y <= half_extent + 1e-4f; y += step) {
        for (float z = -half_extent; z <= half_extent + 1e-4f; z += step) {
            cloud->push_back(pcl::PointXYZ(wall_x, y, z));
        }
    }
    return cloud;
}

std::size_t wallPointCount(float half_extent = 2.0f, float step = kWallStep)
{
    const int n = static_cast<int>(std::lround((2.0f * half_extent) / step)) + 1;
    return static_cast<std::size_t>(n * n);
}

dufomap_submap_filter::KeyframeView makeView(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const Eigen::Matrix4f& pose = identityPose())
{
    dufomap_submap_filter::KeyframeView view;
    view.pose = pose;
    view.cloud = cloud;
    return view;
}

std::vector<dufomap_submap_filter::KeyframeView> movingWallSequence(
    int start_x, int end_x, float half_extent = 2.0f, float step = kWallStep)
{
    std::vector<dufomap_submap_filter::KeyframeView> views;
    views.reserve(static_cast<std::size_t>(end_x - start_x + 1));
    for (int x = start_x; x <= end_x; ++x) {
        views.push_back(makeView(wallCloud(static_cast<float>(x), half_extent, step)));
    }
    return views;
}

struct MovingWallResult {
    std::vector<dufomap_submap_filter::KeyframeView> views;
    dufomap_submap_filter::SubmapResult segmented;
    pcl::PointCloud<pcl::PointXYZ> raw;
};

MovingWallResult runMovingWallScenario(int start_x, int end_x)
{
    MovingWallResult out;
    out.views = movingWallSequence(start_x, end_x);

    dufomap_submap_filter::SubmapFilter filter(defaultConfig());
    for (const auto& view : out.views) {
        filter.integrateKeyframe(view);
    }

    const int last = static_cast<int>(out.views.size()) - 1;
    out.segmented = filter.segmentWindow(out.views, 0, last);
    out.raw = dufomap_submap_filter::buildRawSubmap(out.views, 0, last);
    return out;
}

pcl::PointCloud<pcl::PointXYZRGB> coloredCombined(
    const pcl::PointCloud<pcl::PointXYZ>& static_cloud,
    const pcl::PointCloud<pcl::PointXYZ>& dynamic_cloud,
    const Eigen::Vector3f& origin = Eigen::Vector3f::Zero())
{
    pcl::PointCloud<pcl::PointXYZRGB> combined;
    combined.reserve(static_cloud.size() + dynamic_cloud.size() + 1);

    for (const auto& p : static_cloud.points) {
        pcl::PointXYZRGB c;
        c.x = p.x;
        c.y = p.y;
        c.z = p.z;
        c.r = 80;
        c.g = 200;
        c.b = 80;
        combined.push_back(c);
    }
    for (const auto& p : dynamic_cloud.points) {
        pcl::PointXYZRGB c;
        c.x = p.x;
        c.y = p.y;
        c.z = p.z;
        c.r = 255;
        c.g = 60;
        c.b = 60;
        combined.push_back(c);
    }

    pcl::PointXYZRGB o;
    o.x = origin.x();
    o.y = origin.y();
    o.z = origin.z();
    o.r = 255;
    o.g = 255;
    o.b = 0;
    combined.push_back(o);

    return combined;
}

void saveMovingWallPcds(const MovingWallResult& result, const fs::path& dir)
{
    fs::create_directories(dir);

    const auto combined = coloredCombined(
        result.segmented.static_map, result.segmented.dynamic_cloud);

    const fs::path raw_path = dir / "raw.pcd";
    const fs::path static_path = dir / "static.pcd";
    const fs::path dynamic_path = dir / "dynamic.pcd";
    const fs::path combined_path = dir / "combined_rgb.pcd";

    pcl::io::savePCDFileBinary(raw_path.string(), result.raw);
    pcl::io::savePCDFileBinary(static_path.string(), result.segmented.static_map);
    pcl::io::savePCDFileBinary(dynamic_path.string(), result.segmented.dynamic_cloud);
    pcl::io::savePCDFileBinary(combined_path.string(), combined);

    std::cout << "\nSaved moving-wall clouds to " << dir << ":\n"
              << "  raw.pcd           — all keyframes merged\n"
              << "  static.pcd        — static points (green in combined)\n"
              << "  dynamic.pcd       — dynamic points (red in combined)\n"
              << "  combined_rgb.pcd  — static green + dynamic red + origin yellow\n"
              << "\nVisualize with:\n"
              << "  pcl_viewer " << combined_path << "\n"
              << "  pcl_viewer " << static_path << " " << dynamic_path << "\n";
}

void testMovingWallFirstScanStatic()
{
    std::cout << "\n[test] moving wall: first scan (wall at x=1) is all static\n";

    const auto views = movingWallSequence(1, 1);
    const std::size_t pts_per_wall = wallPointCount();

    dufomap_submap_filter::SubmapFilter filter(defaultConfig());
    filter.integrateKeyframe(views.front());
    const auto result = filter.segmentWindow(views, 0, 0);

    check(result.raw_point_count == pts_per_wall,
          "integrated " + std::to_string(pts_per_wall) + " wall points");
    check(result.static_map.size() == pts_per_wall, "first scan all static");
    check(result.dynamic_cloud.empty(), "no dynamic on first scan");
    check(result.diag_seen_free == 0, "diag_seen_free == 0 on first scan");
}

void testMovingWallAlongX()
{
    std::cout << "\n[test] moving wall: perpendicular wall sweeps x=1..10\n";
    std::cout << "       sensor at origin, wall spans y,z in [-2,2] step 0.2\n";

    constexpr int kStartX = 1;
    constexpr int kEndX = 10;
    const std::size_t pts_per_wall = wallPointCount();
    const std::size_t total_pts = pts_per_wall * static_cast<std::size_t>(kEndX - kStartX + 1);

    const auto scenario = runMovingWallScenario(kStartX, kEndX);
    const auto& result = scenario.segmented;

    std::cout << "       points/wall: " << pts_per_wall
              << "  total: " << total_pts
              << "  static: " << result.static_map.size()
              << "  dynamic: " << result.dynamic_cloud.size()
              << "  diag_seen_free: " << result.diag_seen_free << '\n';

    check(result.raw_point_count == total_pts,
          "segmented all wall points across 10 keyframes");
    check(scenario.raw.size() == total_pts, "raw submap matches total wall points");
    check(result.static_map.size() + result.dynamic_cloud.size() == total_pts,
          "static + dynamic partitions all points");
    check(result.diag_seen_free > 0,
          "some hit voxels were seenFree after wall motion");
    check(!result.dynamic_cloud.empty(),
          "moving wall produces dynamic points (earlier wall positions)");
    check(result.static_map.size() > 0,
          "latest wall positions remain static");

    if (!save_pcd_dir.empty()) {
        saveMovingWallPcds(scenario, save_pcd_dir);
    }
}

void testBuildSubmapMatchesRawPartition()
{
    std::cout << "\n[test] buildSubmap partition matches buildRawSubmap count\n";

    const auto views = movingWallSequence(3, 5);
    const auto raw = dufomap_submap_filter::buildRawSubmap(views, 0, 2);
    const auto filtered = dufomap_submap_filter::buildStaticSubmap(
        views, 0, 2, defaultConfig());

    check(raw.size() == filtered.raw_point_count,
          "raw submap size matches static+dynamic count");
    check(filtered.static_map.size() + filtered.dynamic_cloud.size() == raw.size(),
          "static + dynamic partitions the raw submap");
}

void testIncrementalIntegrateSameAsRebuild()
{
    std::cout << "\n[test] incremental integration matches full rebuild\n";

    const auto views = movingWallSequence(1, 4);

    dufomap_submap_filter::SubmapFilter incremental(defaultConfig());
    for (const auto& view : views) {
        incremental.integrateKeyframe(view);
    }
    const auto inc_result = incremental.segmentWindow(views, 0, 3);

    const auto rebuild_result = dufomap_submap_filter::buildStaticSubmap(
        views, 0, 3, defaultConfig());

    check(inc_result.static_map.size() == rebuild_result.static_map.size(),
          "incremental static count matches rebuild");
    check(inc_result.dynamic_cloud.size() == rebuild_result.dynamic_cloud.size(),
          "incremental dynamic count matches rebuild");
}

}  // namespace

void printUsage(const char* argv0)
{
    std::cout << "Usage: " << argv0 << " [--save-pcd DIR]\n"
              << "  --save-pcd DIR   write raw/static/dynamic/combined PCDs for moving-wall test\n";
}

int main(int argc, char** argv)
{
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--save-pcd" && i + 1 < argc) {
            save_pcd_dir = argv[++i];
        } else if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            return EXIT_SUCCESS;
        } else {
            std::cerr << "Unknown argument: " << arg << '\n';
            printUsage(argv[0]);
            return EXIT_FAILURE;
        }
    }

    std::cout << "dufomap_submap_filter synthetic tests\n";
    std::cout << "Sensor pose: identity (origin). Clouds are in sensor frame.\n";

    testMovingWallFirstScanStatic();
    testMovingWallAlongX();
    testBuildSubmapMatchesRawPartition();
    testIncrementalIntegrateSameAsRebuild();

    std::cout << '\n';
    if (failures == 0) {
        std::cout << "All tests passed.\n";
        return EXIT_SUCCESS;
    }

    std::cout << failures << " test(s) failed.\n";
    return EXIT_FAILURE;
}
