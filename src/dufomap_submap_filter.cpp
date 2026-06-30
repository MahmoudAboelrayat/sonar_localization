#include "sonar_localization/dufomap_submap_filter.hpp"

#include "dufomap/dufomap.h"

#include <pcl/common/transforms.h>

namespace dufomap_submap_filter {

namespace {

ufo::PointCloud toUfoCloud(const pcl::PointCloud<pcl::PointXYZ>& pcl_cloud)
{
    ufo::PointCloud out;
    out.reserve(pcl_cloud.size());
    for (const auto& p : pcl_cloud.points) {
        out.emplace_back(ufo::Point(p.x, p.y, p.z), ufo::DummyType());
    }
    return out;
}

std::vector<float> poseToDufomap(const Eigen::Matrix4f& pose)
{
    Eigen::Quaternionf q(pose.block<3, 3>(0, 0));
    q.normalize();
    return {
        pose(0, 3), pose(1, 3), pose(2, 3),
        q.w(), q.x(), q.y(), q.z()};
}

SubmapResult segmentKeyframes(
    dufomap::MapUpdater& map_updater,
    const std::vector<KeyframeView>& keyframes,
    int start_idx,
    int end_idx,
    bool collect_diagnostics)
{
    SubmapResult result;
    if (start_idx > end_idx || keyframes.empty()) {
        return result;
    }

    for (int i = start_idx; i <= end_idx; ++i) {
        const auto& kf = keyframes[static_cast<std::size_t>(i)];
        if (!kf.cloud || kf.cloud->empty()) {
            continue;
        }

        ufo::PointCloud ufo_cloud = toUfoCloud(*kf.cloud);
        const std::vector<float> pose = poseToDufomap(kf.pose);
        if (collect_diagnostics) {
            const dufomap::SegmentDiagnostics diag =
                map_updater.countSegmentDiagnostics(ufo_cloud, pose, true);
            result.diag_seen_free += diag.seen_free;
            result.diag_label_valid += diag.label_valid;
        }

        const std::vector<bool> labels =
            map_updater.segmentPoints(ufo_cloud, pose, true);

        pcl::PointCloud<pcl::PointXYZ> cloud_world;
        pcl::transformPointCloud(*kf.cloud, cloud_world, kf.pose);

        for (std::size_t j = 0; j < labels.size(); ++j) {
            const auto& pt = cloud_world.points[j];
            if (labels[j]) {
                result.dynamic_cloud.push_back(pt);
            } else {
                result.static_map.push_back(pt);
            }
        }
    }

    result.raw_point_count =
        result.static_map.size() + result.dynamic_cloud.size();
    return result;
}

}  // namespace

struct SubmapFilter::Impl {
    std::unique_ptr<dufomap::MapUpdater> updater;
};

SubmapFilter::SubmapFilter(const Config& config)
    : impl_(std::make_unique<Impl>()),
      config_(config)
{
    impl_->updater = std::make_unique<dufomap::MapUpdater>(
        config.resolution,
        config.d_s,
        config.d_p,
        config.num_threads,
        config.hit_extension,
        config.ray_passthrough_hits);
    impl_->updater->setMapPropagate(config.propagate);
}

SubmapFilter::~SubmapFilter() = default;

void SubmapFilter::reset()
{
    impl_->updater->clean();
}

void SubmapFilter::integrateKeyframe(const KeyframeView& keyframe)
{
    if (!keyframe.cloud || keyframe.cloud->empty()) {
        return;
    }
    ufo::PointCloud ufo_cloud = toUfoCloud(*keyframe.cloud);
    impl_->updater->run(ufo_cloud, poseToDufomap(keyframe.pose), true);
}

SubmapResult SubmapFilter::segmentWindow(
    const std::vector<KeyframeView>& keyframes,
    int start_idx,
    int end_idx)
{
    if (config_.propagate || config_.use_cluster) {
        impl_->updater->oncePropagateCluster(config_.propagate, config_.use_cluster);
    }
    return segmentKeyframes(*impl_->updater, keyframes, start_idx, end_idx, true);
}

SubmapResult SubmapFilter::buildSubmap(
    const std::vector<KeyframeView>& keyframes,
    int start_idx,
    int end_idx)
{
    reset();
    for (int i = start_idx; i <= end_idx; ++i) {
        integrateKeyframe(keyframes[static_cast<std::size_t>(i)]);
    }
    return segmentWindow(keyframes, start_idx, end_idx);
}

pcl::PointCloud<pcl::PointXYZ> buildRawSubmap(
    const std::vector<KeyframeView>& keyframes,
    int start_idx,
    int end_idx)
{
    pcl::PointCloud<pcl::PointXYZ> merged;
    if (start_idx > end_idx || keyframes.empty()) {
        return merged;
    }

    for (int i = start_idx; i <= end_idx; ++i) {
        if (!keyframes[static_cast<std::size_t>(i)].cloud ||
            keyframes[static_cast<std::size_t>(i)].cloud->empty()) {
            continue;
        }
        pcl::PointCloud<pcl::PointXYZ> transformed;
        pcl::transformPointCloud(
            *keyframes[static_cast<std::size_t>(i)].cloud,
            transformed,
            keyframes[static_cast<std::size_t>(i)].pose);
        merged += transformed;
    }
    return merged;
}

SubmapResult buildStaticSubmap(
    const std::vector<KeyframeView>& keyframes,
    int start_idx,
    int end_idx,
    const Config& config)
{
    SubmapFilter filter(config);
    return filter.buildSubmap(keyframes, start_idx, end_idx);
}

}  // namespace dufomap_submap_filter
