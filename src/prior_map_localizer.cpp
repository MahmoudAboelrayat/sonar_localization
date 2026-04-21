// MIT License
// Prior Map Localizer — Implementation
// =====================================

#include "prior_map_localizer.hpp"
#include <pcl/common/transforms.h>

// ────────────────────────────────────────────────────────────
//  Constructor
// ────────────────────────────────────────────────────────────
PriorMapLocalizer::PriorMapLocalizer()
    : Node("prior_map_localizer")
{
    // ── Declare all parameters ────────────────────────────
    // Frames
    odom_frame_      = declare_parameter<std::string>("frames.odom_frame",      "odom");
    base_frame_      = declare_parameter<std::string>("frames.base_frame",      "saabmarine/base_link");
    sonar_frame_     = declare_parameter<std::string>("frames.sonar_frame",     "saabmarine/sonar_link");
    prior_map_frame_ = declare_parameter<std::string>("frames.prior_map_frame", "unity_origin");

    // Initial map→odom transform (set to known starting position)
    double init_x   = declare_parameter<double>("init_pose.x",   0.0);
    double init_y   = declare_parameter<double>("init_pose.y",   0.0);
    double init_z   = declare_parameter<double>("init_pose.z",   0.0);
    double init_yaw = declare_parameter<double>("init_pose.yaw", 0.0);

    Eigen::AngleAxisf init_rot(static_cast<float>(init_yaw), Eigen::Vector3f::UnitZ());
    map_T_odom_.setIdentity();
    map_T_odom_.block<3,3>(0,0) = init_rot.toRotationMatrix();
    map_T_odom_.block<3,1>(0,3) = Eigen::Vector3f(init_x, init_y, init_z);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Prior map
    std::string map_path = declare_parameter<std::string>(
        "prior_map_path", "gt_map.pcd");

    // Sonar extrinsics (sonar → base_link)
    double s2b_x   = declare_parameter<double>("tf.sonar2base_x",     -0.111);
    double s2b_y   = declare_parameter<double>("tf.sonar2base_y",      0.000);
    double s2b_z   = declare_parameter<double>("tf.sonar2base_z",      0.249);
    double s2b_r   = declare_parameter<double>("tf.sonar2base_roll",   0.0);
    double s2b_p   = declare_parameter<double>("tf.sonar2base_pitch",  34.0);
    double s2b_yaw = declare_parameter<double>("tf.sonar2base_yaw",    0.0);

    pc_topic_ = declare_parameter<std::string>("topics.point_cloud_topic",  "/sonar/point_cloud_local");
    odom_topic_ = declare_parameter<std::string>("topics.odom_topic",  "/odometry/filtered");


    // Local VGICP (scan → local submap)
    local_max_dist_   = declare_parameter<double>("local_vgicp.max_dist",   1.5);
    local_epsilon_    = declare_parameter<double>("local_vgicp.epsilon",    1e-6);
    local_max_iter_   = declare_parameter<int>   ("local_vgicp.max_iter",   150);
    local_resolution_ = declare_parameter<double>("local_vgicp.resolution", 0.15);

    // Global VGICP (local submap → prior map)
    global_max_dist_   = declare_parameter<double>("global_vgicp.max_dist",    2.0);
    global_epsilon_    = declare_parameter<double>("global_vgicp.epsilon",     1e-6);
    global_max_iter_   = declare_parameter<int>   ("global_vgicp.max_iter",    200);
    global_resolution_ = declare_parameter<double>("global_vgicp.resolution",  0.1);
    score_thresh_      = declare_parameter<double>("global_vgicp.score_thresh", 0.15);
    match_every_n_kf_  = declare_parameter<int>   ("global_vgicp.match_every_n_kf", 3);

    prior_map_resolution_ = declare_parameter<double>("prior_map_resolution",  0.5);


    // Keyframe / submap
    kf_dist_thresh_  = declare_parameter<double>("keyframe.dist_thresh",  0.3);
    kf_angle_thresh_ = declare_parameter<double>("keyframe.angle_thresh", 4.0);
    submap_size_     = declare_parameter<int>   ("keyframe.submap_size",  15);

    // Outlier removal
    use_sor_           = declare_parameter<bool>  ("outlier_removal.use_sor",       true);
    sor_neighbors_     = declare_parameter<int>   ("outlier_removal.num_neighbors",  20);
    sor_stddev_        = declare_parameter<double>("outlier_removal.stddev_thresh",   1.0);
    use_radius_        = declare_parameter<bool>  ("outlier_removal.use_radius",     true);
    radius_search_     = declare_parameter<double>("outlier_removal.radius_search",   0.3);
    radius_min_neighbors_ = declare_parameter<int>("outlier_removal.min_neighbors",  15);

    // Intensity filter
    use_intensity_filter_ = declare_parameter<bool> ("intensity_filter.enabled",  false);
    min_intensity_        = declare_parameter<float>("intensity_filter.min_value", 0.005f);

    // ── Build sonar→base_link transform ──────────────────
    Eigen::AngleAxisf rx(static_cast<float>(s2b_r   * M_PI/180.0), Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf ry(static_cast<float>(s2b_p   * M_PI/180.0), Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rz(static_cast<float>(s2b_yaw * M_PI/180.0), Eigen::Vector3f::UnitZ());
    Eigen::Quaternionf q_sb = rz * ry * rx;

    sonar2base_.setIdentity();
    sonar2base_.block<3,3>(0,0) = q_sb.toRotationMatrix();
    sonar2base_.block<3,1>(0,3) = Eigen::Vector3f(s2b_x, s2b_y, s2b_z);
    base2sonar_ = sonar2base_.inverse();

    RCLCPP_INFO(get_logger(), "Sonar→base transform loaded.");

    // ── Load prior map ────────────────────────────────────
    prior_map_.reset(new CloudT);
    if (pcl::io::loadPCDFile<PointT>(map_path, *prior_map_) < 0) {
        RCLCPP_ERROR(get_logger(),
            "Failed to load prior map from: %s", map_path.c_str());
        throw std::runtime_error("Prior map load failed");
    }
    RCLCPP_INFO(get_logger(),
        "Loaded prior map: %zu points from %s",
        prior_map_->size(), map_path.c_str());

    // Downsample prior map to global VGICP resolution
    voxelFilter(prior_map_, static_cast<float>(prior_map_resolution_));
    RCLCPP_INFO(get_logger(),
        "Prior map downsampled: %zu points", prior_map_->size());

    // ── Configure VGICP instances ─────────────────────────

    // Local VGICP — scan to submap (run every frame)
    local_vgicp_.setNumThreads(4);
    local_vgicp_.setMaxCorrespondenceDistance(local_max_dist_);
    local_vgicp_.setTransformationEpsilon(local_epsilon_);
    local_vgicp_.setMaximumIterations(local_max_iter_);
    local_vgicp_.setResolution(local_resolution_);

    // Global VGICP — submap to prior (run every N keyframes)
    global_vgicp_.setNumThreads(4);
    global_vgicp_.setMaxCorrespondenceDistance(global_max_dist_);
    global_vgicp_.setTransformationEpsilon(global_epsilon_);
    global_vgicp_.setMaximumIterations(global_max_iter_);
    global_vgicp_.setResolution(global_resolution_);
    global_vgicp_.setInputTarget(prior_map_);  // ← fixed forever

    // ── Filters ───────────────────────────────────────────
    map_filter_.setLeafSize(
        static_cast<float>(local_resolution_),
        static_cast<float>(local_resolution_),
        static_cast<float>(local_resolution_));

    sor_filter_.setMeanK(sor_neighbors_);
    sor_filter_.setStddevMulThresh(sor_stddev_);

    radius_filter_.setRadiusSearch(radius_search_);
    radius_filter_.setMinNeighborsInRadius(radius_min_neighbors_);

    // ── Initialize containers ─────────────────────────────
    local_map_.reset(new CloudT);

    // ── Publishers ────────────────────────────────────────
    local_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "local_map", 1);
    prior_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "prior_map", 1);
    match_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "match_markers", 10);

    // ── Subscribers ───────────────────────────────────────
    ekf_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 10,
        std::bind(&PriorMapLocalizer::ekfCallback, this,
                  std::placeholders::_1));

    pc_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        pc_topic_, 10,
        std::bind(&PriorMapLocalizer::cloudCallback, this,
                  std::placeholders::_1));

    initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 1,
        std::bind(&PriorMapLocalizer::initialPoseCallback, this,
                  std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
        "Prior map localizer initialized.\n"
        "  Odom frame:  %s\n"
        "  Base frame:  %s\n"
        "  Score thresh: %.3f\n"
        "  Match every: %d keyframes",
        odom_frame_.c_str(), base_frame_.c_str(),
        score_thresh_, match_every_n_kf_);

    // Publish prior map + map→odom TF on a timer so RViz always has them
    prior_map_timer_ = create_wall_timer(
        std::chrono::milliseconds(200),
        [this]() {
            auto t = this->now();
            publishPriorMap(t);
            publishMapToOdomTF(t);
        });

    // Publish immediately once
    publishPriorMap(this->now());
    publishMapToOdomTF(this->now());
}

// ────────────────────────────────────────────────────────────
//  EKF callback — store latest dead reckoning pose
void PriorMapLocalizer::initialPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    auto& p = msg->pose.pose;
    Eigen::Quaternionf q(p.orientation.w, p.orientation.x,
                          p.orientation.y, p.orientation.z);
    map_T_odom_.setIdentity();
    map_T_odom_.block<3,3>(0,0) = q.toRotationMatrix();
    map_T_odom_.block<3,1>(0,3) = Eigen::Vector3f(
        p.position.x, p.position.y, p.position.z);

    // Reset match state so next global VGICP starts fresh from this pose
    has_global_match_ = false;
    map_initialized_  = false;
    keyframes_.clear();
    local_map_->clear();

    publishMapToOdomTF(this->now());
    RCLCPP_INFO(get_logger(),
        "Initial pose set from RViz: [%.2f, %.2f, %.2f]",
        p.position.x, p.position.y, p.position.z);
}

// ────────────────────────────────────────────────────────────
void PriorMapLocalizer::ekfCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg)
{
    Eigen::Quaternionf q(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z);
    Eigen::Vector3f t(
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z);

    std::lock_guard<std::mutex> lock(ekf_mutex_);
    ekf_pose_.setIdentity();
    ekf_pose_.block<3,3>(0,0) = q.toRotationMatrix();
    ekf_pose_.block<3,1>(0,3) = t;
    has_ekf_ = true;
}

// ────────────────────────────────────────────────────────────
//  Point cloud callback — main pipeline entry point
// ────────────────────────────────────────────────────────────
void PriorMapLocalizer::cloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    if (!has_ekf_) {
        RCLCPP_WARN_ONCE(get_logger(), "Waiting for first EKF message...");
        return;
    }

    auto t_start = this->now();

    // ── Convert and preprocess ────────────────────────────
    CloudTPtr raw(new CloudT);
    pcl::fromROSMsg(*msg, *raw);

    CloudTPtr scan = preprocessCloud(raw);
    if (!scan || scan->empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "Empty scan after preprocessing.");
        return;
    }

    // ── Get current EKF pose ──────────────────────────────
    Eigen::Matrix4f ekf_pose;
    {
        std::lock_guard<std::mutex> lock(ekf_mutex_);
        ekf_pose = ekf_pose_;
    }

    // ── Step 1: Build local map ───────────────────────────
    if (!buildLocalMap(scan, ekf_pose, rclcpp::Time(msg->header.stamp)))
        return;

    scan_count_++;

    // ── Step 2: Match local map to prior every N keyframes ─
    int kf_count = static_cast<int>(keyframes_.size());
    if (kf_count > 0 && kf_count % match_every_n_kf_ == 0) {
        matchLocalMapToPrior(rclcpp::Time(msg->header.stamp));
    }

    // ── Publish local map periodically ───────────────────
    if (scan_count_ % 5 == 0) {
        publishLocalMap(rclcpp::Time(msg->header.stamp));
    }

    double elapsed = (this->now() - t_start).seconds();
    RCLCPP_DEBUG(get_logger(),
        "Scan processed in %.3fs | KFs: %d | Map: %zu pts",
        elapsed, kf_count, local_map_->size());
}

// ────────────────────────────────────────────────────────────
//  Step 1: Build local map via scan-to-submap VGICP
// ────────────────────────────────────────────────────────────
bool PriorMapLocalizer::buildLocalMap(
    const CloudTPtr& scan,
    const Eigen::Matrix4f& ekf_pose,
    const rclcpp::Time& stamp)
{
    // First frame — initialize with EKF pose
    if (!map_initialized_) {
        global_pose_   = ekf_pose;
        prev_ekf_pose_ = ekf_pose;
        addKeyframe(scan, global_pose_, stamp);
        updateSubmap();
        map_initialized_ = true;
        RCLCPP_INFO(get_logger(), "Local map initialized at EKF origin.");
        return true;
    }

    // ── EKF delta as initial guess ────────────────────────
    Eigen::Matrix4f ekf_delta    = prev_ekf_pose_.inverse() * ekf_pose;
    Eigen::Matrix4f initial_guess = global_pose_ * ekf_delta;

    // Sanity check — reject huge deltas (EKF jump or initialization issue)
    float delta_dist  = ekf_delta.block<3,1>(0,3).norm();
    Eigen::AngleAxisf aa(Eigen::Matrix3f(ekf_delta.block<3,3>(0,0)));
    float delta_angle = std::abs(aa.angle()) * 180.0f / M_PI;

    if (delta_dist > 2.0f || delta_angle > 45.0f) {
        RCLCPP_WARN(get_logger(),
            "Large EKF delta (%.2fm, %.1fdeg) — using last pose as guess",
            delta_dist, delta_angle);
        initial_guess = global_pose_;
    }

    // ── Scan-to-local-submap VGICP ────────────────────────
    local_vgicp_.setInputTarget(local_map_);
    local_vgicp_.setInputSource(scan);

    CloudT aligned;
    local_vgicp_.align(aligned, initial_guess);

    if (!local_vgicp_.hasConverged()) {
        RCLCPP_WARN(get_logger(), "Local VGICP did not converge — falling back to EKF.");
        global_pose_   = initial_guess;
        prev_ekf_pose_ = ekf_pose;
        return true;
    }

    double score = local_vgicp_.getFitnessScore();
    global_pose_   = local_vgicp_.getFinalTransformation();
    prev_ekf_pose_ = ekf_pose;

    RCLCPP_DEBUG(get_logger(),
        "Local VGICP: score=%.4f  pos=[%.2f, %.2f, %.2f]",
        score,
        global_pose_(0,3), global_pose_(1,3), global_pose_(2,3));

    // ── Add keyframe if moved enough ──────────────────────
    if (shouldAddKeyframe(global_pose_)) {
        addKeyframe(scan, global_pose_, stamp);
        updateSubmap();
    }

    return true;
}

// ────────────────────────────────────────────────────────────
//  Keyframe management
// ────────────────────────────────────────────────────────────
bool PriorMapLocalizer::shouldAddKeyframe(const Eigen::Matrix4f& pose)
{
    if (keyframes_.empty()) return true;

    Eigen::Matrix4f delta = keyframes_.back().pose.inverse() * pose;
    float dist  = delta.block<3,1>(0,3).norm();
    Eigen::AngleAxisf aa(Eigen::Matrix3f(delta.block<3,3>(0,0)));
    float angle = std::abs(aa.angle()) * 180.0f / M_PI;

    return dist  > static_cast<float>(kf_dist_thresh_) ||
           angle > static_cast<float>(kf_angle_thresh_);
}

void PriorMapLocalizer::addKeyframe(
    const CloudTPtr& scan,
    const Eigen::Matrix4f& pose,
    const rclcpp::Time& stamp)
{
    Keyframe kf;
    kf.id    = static_cast<int>(keyframes_.size());
    kf.pose  = pose;
    kf.cloud = scan;
    kf.stamp = stamp;
    keyframes_.push_back(kf);

    RCLCPP_INFO(get_logger(),
        "Keyframe %d added | total: %zu | pos=[%.2f, %.2f, %.2f]",
        kf.id, keyframes_.size(),
        pose(0,3), pose(1,3), pose(2,3));
}

void PriorMapLocalizer::updateSubmap()
{
    local_map_->clear();

    int start = std::max(0, static_cast<int>(keyframes_.size()) - submap_size_);
    for (int i = start; i < static_cast<int>(keyframes_.size()); ++i) {
        CloudT transformed;
        pcl::transformPointCloud(*keyframes_[i].cloud, transformed, keyframes_[i].pose);
        *local_map_ += transformed;
    }

    voxelFilter(local_map_, static_cast<float>(local_resolution_));

    RCLCPP_DEBUG(get_logger(),
        "Submap updated: %zu pts from %d keyframes",
        local_map_->size(),
        static_cast<int>(keyframes_.size()) - start);
}

// ────────────────────────────────────────────────────────────
//  Step 2: Match local submap against fixed prior map
// ────────────────────────────────────────────────────────────
void PriorMapLocalizer::matchLocalMapToPrior(const rclcpp::Time& stamp)
{
    if (local_map_->empty()) return;

    // ── Transform local map from odom_frame into prior_map_frame ─
    // map_T_odom_ is our current best estimate of map→odom.
    // Applying it to the local map (in odom frame) gives us
    // the submap expressed in the prior map frame so VGICP
    // can compare apples to apples.
    CloudTPtr local_in_map(new CloudT);
    pcl::transformPointCloud(*local_map_, *local_in_map, map_T_odom_);

    // ── Global VGICP (both clouds now in prior_map_frame) ─────
    global_vgicp_.setInputSource(local_in_map);

    // Initial guess: identity — the pre-transform already placed
    // the source near the target. Use last correction as refinement seed.
    CloudT aligned;
    global_vgicp_.align(aligned, Eigen::Matrix4f::Identity());

    if (!global_vgicp_.hasConverged()) {
        RCLCPP_WARN(get_logger(), "Global VGICP did not converge.");
        publishMapToOdomTF(stamp);  // keep publishing last known TF
        return;
    }

    double score = global_vgicp_.getFitnessScore();

    RCLCPP_INFO(get_logger(),
        "Global match: score=%.4f (thresh=%.4f)  pos=[%.2f, %.2f, %.2f]",
        score, score_thresh_,
        global_vgicp_.getFinalTransformation()(0,3),
        global_vgicp_.getFinalTransformation()(1,3),
        global_vgicp_.getFinalTransformation()(2,3));

    if (score > score_thresh_) {
        RCLCPP_WARN(get_logger(),
            "Poor global match (%.4f > %.4f) — skipping correction.",
            score, score_thresh_);
        publishMapToOdomTF(stamp);
        return;
    }

    // ── VGICP result = small correction on top of map_T_odom_ ─
    // T_correction * map_T_odom_old * local_odom = prior_map
    // → new map_T_odom_ = T_correction * map_T_odom_old
    Eigen::Matrix4f correction = global_vgicp_.getFinalTransformation();
    map_T_odom_ = correction * map_T_odom_;

    has_global_match_ = true;

    RCLCPP_INFO(get_logger(),
        "✓ map→odom updated. pos=[%.2f, %.2f, %.2f]",
        map_T_odom_(0,3), map_T_odom_(1,3), map_T_odom_(2,3));

    publishMatchMarker(map_T_odom_, stamp, score);
    publishMapToOdomTF(stamp);
}

// ────────────────────────────────────────────────────────────
//  Preprocessing
// ────────────────────────────────────────────────────────────
CloudTPtr PriorMapLocalizer::preprocessCloud(const CloudTPtr& raw)
{
    if (!raw || raw->empty()) return nullptr;

    // ── 1. Intensity filter ───────────────────────────────
    CloudTPtr intensity_filtered(new CloudT);
    if (use_intensity_filter_) {
        intensity_filtered->reserve(raw->size());
        for (const auto& pt : *raw) {
            if (std::isfinite(pt.x) && pt.intensity > min_intensity_)
                intensity_filtered->push_back(pt);
        }
    } else {
        for (const auto& pt : *raw) {
            if (std::isfinite(pt.x))
                intensity_filtered->push_back(pt);
        }
    }

    if (intensity_filtered->empty()) return nullptr;

    // ── 2. Transform sonar → base_link ───────────────────
    CloudTPtr in_base(new CloudT);
    pcl::transformPointCloud(*intensity_filtered, *in_base, base2sonar_);

    // ── 3. Statistical outlier removal ───────────────────
    CloudTPtr sor_filtered(new CloudT);
    if (use_sor_ && in_base->size() > static_cast<size_t>(sor_neighbors_)) {
        sor_filter_.setInputCloud(in_base);
        sor_filter_.filter(*sor_filtered);
    } else {
        sor_filtered = in_base;
    }

    // ── 4. Radius outlier removal ─────────────────────────
    CloudTPtr clean(new CloudT);
    if (use_radius_ && sor_filtered->size() > 10) {
        radius_filter_.setInputCloud(sor_filtered);
        radius_filter_.filter(*clean);
    } else {
        clean = sor_filtered;
    }

    if (clean->empty()) {
        RCLCPP_DEBUG(get_logger(), "Scan empty after preprocessing.");
        return nullptr;
    }

    return clean;
}

void PriorMapLocalizer::voxelFilter(CloudTPtr& cloud, float leaf_size)
{
    if (!cloud || cloud->empty()) return;
    pcl::VoxelGrid<PointT> vg;
    vg.setLeafSize(leaf_size, leaf_size, leaf_size);
    vg.setInputCloud(cloud);
    CloudTPtr out(new CloudT);
    vg.filter(*out);
    cloud = out;
}

// ────────────────────────────────────────────────────────────
//  Publishers
// ────────────────────────────────────────────────────────────

void PriorMapLocalizer::publishLocalMap(const rclcpp::Time& stamp)
{
    if (local_map_->empty()) return;

    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*local_map_, msg);
    msg.header.frame_id = odom_frame_;
    msg.header.stamp    = stamp;
    local_map_pub_->publish(msg);
}

void PriorMapLocalizer::publishPriorMap(const rclcpp::Time& stamp)
{
    if (!prior_map_ || prior_map_->empty()) return;

    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*prior_map_, msg);
    msg.header.frame_id = prior_map_frame_;
    msg.header.stamp    = rclcpp::Time(0);  // use latest TF, avoids sim/wall clock mismatch
    prior_map_pub_->publish(msg);

    // RCLCPP_INFO(get_logger(),
    //     "Prior map published (%zu pts) on /prior_map (latched)",
    //     prior_map_->size());
}

void PriorMapLocalizer::publishMatchMarker(
    const Eigen::Matrix4f& pose,
    const rclcpp::Time& stamp,
    double score)
{
    visualization_msgs::msg::MarkerArray ma;

    // Sphere at the match position
    visualization_msgs::msg::Marker sphere;
    sphere.header.frame_id = prior_map_frame_;
    sphere.header.stamp    = stamp;
    sphere.ns              = "global_match";
    sphere.id              = 0;
    sphere.type            = visualization_msgs::msg::Marker::SPHERE;
    sphere.action          = visualization_msgs::msg::Marker::ADD;
    sphere.pose.position.x = pose(0, 3);
    sphere.pose.position.y = pose(1, 3);
    sphere.pose.position.z = pose(2, 3);
    sphere.pose.orientation.w = 1.0;
    sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.5;
    // Green = good match, yellow = borderline
    double t = std::min(score / score_thresh_, 1.0);
    sphere.color.r = static_cast<float>(t);
    sphere.color.g = 1.0f;
    sphere.color.b = 0.0f;
    sphere.color.a = 1.0f;
    sphere.lifetime = rclcpp::Duration::from_seconds(0);  // persistent
    ma.markers.push_back(sphere);

    // Arrow showing heading (X axis of the matched pose)
    visualization_msgs::msg::Marker arrow;
    arrow.header  = sphere.header;
    arrow.ns      = "global_match";
    arrow.id      = 1;
    arrow.type    = visualization_msgs::msg::Marker::ARROW;
    arrow.action  = visualization_msgs::msg::Marker::ADD;
    Eigen::Quaternionf q(pose.block<3,3>(0,0));
    arrow.pose.position    = sphere.pose.position;
    arrow.pose.orientation.x = q.x();
    arrow.pose.orientation.y = q.y();
    arrow.pose.orientation.z = q.z();
    arrow.pose.orientation.w = q.w();
    arrow.scale.x = 1.0;   // shaft length
    arrow.scale.y = 0.1;   // shaft diameter
    arrow.scale.z = 0.15;  // head diameter
    arrow.color   = sphere.color;
    arrow.lifetime = rclcpp::Duration::from_seconds(0);
    ma.markers.push_back(arrow);

    match_marker_pub_->publish(ma);
}

void PriorMapLocalizer::publishMapToOdomTF(const rclcpp::Time& stamp)
{
    Eigen::Quaternionf q(map_T_odom_.block<3,3>(0,0));

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp    = stamp;
    t.header.frame_id = prior_map_frame_;   // parent = map
    t.child_frame_id  = odom_frame_;        // child  = odom
    t.transform.translation.x = map_T_odom_(0,3);
    t.transform.translation.y = map_T_odom_(1,3);
    t.transform.translation.z = map_T_odom_(2,3);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(t);
}

// ────────────────────────────────────────────────────────────
//  Main
// ────────────────────────────────────────────────────────────
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<PriorMapLocalizer>());
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("prior_map_localizer"),
            "Fatal error: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}