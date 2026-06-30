#pragma once

#include <cstddef>
#include <memory>
#include <vector>

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace dufomap_submap_filter {

struct KeyframeView {
    // Sensor pose in odom and points in the sensor (sonar) frame.
    Eigen::Matrix4f pose;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
};

struct Config {
    double resolution{0.2};
    double d_s{0.1};
    std::size_t d_p{2};
    std::size_t num_threads{4};
    bool hit_extension{true};
    bool ray_passthrough_hits{false};
    bool use_cluster{false};
    bool propagate{true};
};

struct SubmapResult {
    pcl::PointCloud<pcl::PointXYZ> static_map;
    pcl::PointCloud<pcl::PointXYZ> dynamic_cloud;
    std::size_t raw_point_count{0};
    std::size_t diag_seen_free{0};
    std::size_t diag_label_valid{0};
};

class SubmapFilter {
public:
    explicit SubmapFilter(const Config& config);
    ~SubmapFilter();

    SubmapFilter(const SubmapFilter&) = delete;
    SubmapFilter& operator=(const SubmapFilter&) = delete;

    void reset();
    void integrateKeyframe(const KeyframeView& keyframe);
    SubmapResult segmentWindow(
        const std::vector<KeyframeView>& keyframes,
        int start_idx,
        int end_idx);

    SubmapResult buildSubmap(
        const std::vector<KeyframeView>& keyframes,
        int start_idx,
        int end_idx);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    Config config_;
};

pcl::PointCloud<pcl::PointXYZ> buildRawSubmap(
    const std::vector<KeyframeView>& keyframes,
    int start_idx,
    int end_idx);

SubmapResult buildStaticSubmap(
    const std::vector<KeyframeView>& keyframes,
    int start_idx,
    int end_idx,
    const Config& config);

}  // namespace dufomap_submap_filter
