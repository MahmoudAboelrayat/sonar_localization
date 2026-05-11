// MIT License
// Prior Map Localizer with GTSAM — Implementation
// =================================================
// What changed from the original:
//   1. addKeyframe()        → also adds node + odometry factor to GTSAM graph
//   2. matchLocalMapToPrior() → adds PriorFactor instead of direct correction
//   3. optimizeGraph()       → NEW: iSAM2 optimization after each new factor
//   4. updateKeyframePosesFromGraph() → NEW: syncs keyframe poses from GTSAM
//   5. map_T_odom_           → now derived from GTSAM result, not set directly

#include "prior_map_localizer.hpp"
#include <pcl/common/transforms.h>

// ────────────────────────────────────────────────────────────
//  Constructor
// ────────────────────────────────────────────────────────────
PriorMapLocalizer::PriorMapLocalizer()
    : Node("prior_map_localizer")
{
    // ── Frames ────────────────────────────────────────────
    odom_frame_      = declare_parameter<std::string>("frames.odom_frame",      "odom");
    base_frame_      = declare_parameter<std::string>("frames.base_frame",      "saabmarine/base_link");
    sonar_frame_     = declare_parameter<std::string>("frames.sonar_frame",     "saabmarine/sonar_link");
    prior_map_frame_ = declare_parameter<std::string>("frames.prior_map_frame", "unity_origin");

    // ── Initial pose (map → odom) ─────────────────────────
    double init_x   = declare_parameter<double>("init_pose.x",   0.0);
    double init_y   = declare_parameter<double>("init_pose.y",   0.0);
    double init_z   = declare_parameter<double>("init_pose.z",   0.0);
    double init_yaw = declare_parameter<double>("init_pose.yaw", 0.0);

    Eigen::AngleAxisf init_rot(
        static_cast<float>(init_yaw), Eigen::Vector3f::UnitZ());
    map_T_odom_.setIdentity();
    map_T_odom_.block<3,3>(0,0) = init_rot.toRotationMatrix();
    map_T_odom_.block<3,1>(0,3) = Eigen::Vector3f(init_x, init_y, init_z);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // ── Prior map subscription ────────────────────────────
    std::string prior_map_topic = declare_parameter<std::string>(
        "topics.prior_map_topic", "prior_map");

    // ── Extrinsics ────────────────────────────────────────
    double s2b_x   = declare_parameter<double>("tf.sonar2base_x",     -0.111);
    double s2b_y   = declare_parameter<double>("tf.sonar2base_y",      0.000);
    double s2b_z   = declare_parameter<double>("tf.sonar2base_z",      0.249);
    double s2b_r   = declare_parameter<double>("tf.sonar2base_roll",   0.0);
    double s2b_p   = declare_parameter<double>("tf.sonar2base_pitch",  34.0);
    double s2b_yaw = declare_parameter<double>("tf.sonar2base_yaw",    0.0);

    pc_topic_   = declare_parameter<std::string>("topics.point_cloud_topic", "/sonar/point_cloud_local");
    odom_topic_ = declare_parameter<std::string>("topics.odom_topic",        "/odometry/filtered");

    // ── VGICP params ──────────────────────────────────────
    local_max_dist_   = declare_parameter<double>("local_vgicp.max_dist",   1.5);
    local_epsilon_    = declare_parameter<double>("local_vgicp.epsilon",    1e-6);
    local_max_iter_   = declare_parameter<int>   ("local_vgicp.max_iter",   150);
    local_resolution_ = declare_parameter<double>("local_vgicp.resolution", 0.15);

    global_max_dist_   = declare_parameter<double>("global_vgicp.max_dist",    2.0);
    global_epsilon_    = declare_parameter<double>("global_vgicp.epsilon",     1e-6);
    global_max_iter_   = declare_parameter<int>   ("global_vgicp.max_iter",    200);
    global_resolution_ = declare_parameter<double>("global_vgicp.resolution",  0.1);
    score_thresh_      = declare_parameter<double>("global_vgicp.score_thresh", 0.15);
    match_every_n_kf_  = declare_parameter<int>   ("global_vgicp.match_every_n_kf", 3);


    // ── Keyframe ──────────────────────────────────────────
    kf_dist_thresh_  = declare_parameter<double>("keyframe.dist_thresh",  0.3);
    kf_angle_thresh_ = declare_parameter<double>("keyframe.angle_thresh", 4.0);
    submap_size_      = declare_parameter<int>   ("keyframe.submap_size",   15);
    submap_voxel_res_ = declare_parameter<double>("keyframe.submap_voxel_res", 0.2);

    // ── Outlier removal ───────────────────────────────────
    use_sor_              = declare_parameter<bool>  ("outlier_removal.use_sor",       true);
    sor_neighbors_        = declare_parameter<int>   ("outlier_removal.num_neighbors",  20);
    sor_stddev_           = declare_parameter<double>("outlier_removal.stddev_thresh",   1.0);
    use_radius_           = declare_parameter<bool>  ("outlier_removal.use_radius",     true);
    radius_search_        = declare_parameter<double>("outlier_removal.radius_search",   0.3);
    radius_min_neighbors_ = declare_parameter<int>   ("outlier_removal.min_neighbors",  15);

    use_intensity_filter_ = declare_parameter<bool> ("intensity_filter.enabled",  false);
    min_intensity_        = declare_parameter<float>("intensity_filter.min_value", 0.005f);

    use_dbscan_          = declare_parameter<bool>  ("dbscan.enabled",        false);
    dbscan_tolerance_    = declare_parameter<double>("dbscan.tolerance",       1.0);
    dbscan_min_pts_      = declare_parameter<int>   ("dbscan.min_points",      10);
    dbscan_keep_clusters_= declare_parameter<int>   ("dbscan.keep_clusters",   1);

    // ── Global matcher selection ──────────────────────────
    use_ndt_        = declare_parameter<bool>  ("global_matcher.use_ndt",     false);
    ndt_resolution_ = declare_parameter<double>("global_matcher.ndt_resolution", 2.0);
    ndt_step_size_  = declare_parameter<double>("global_matcher.ndt_step_size",  0.5);
    ndt_epsilon_    = declare_parameter<double>("global_matcher.ndt_epsilon",    0.01);
    ndt_max_iter_   = declare_parameter<int>   ("global_matcher.ndt_max_iter",   50);

    // ── GTSAM noise parameters ────────────────────────────
    // Odometry: how much we trust the local VGICP (scan-to-submap)
    // Larger = less trust = GTSAM can deviate more from odometry
    odom_noise_trans_      = declare_parameter<double>("gtsam.odom_noise_trans",      0.1);
    odom_noise_rot_        = declare_parameter<double>("gtsam.odom_noise_rot",        0.05);
    // Prior map: noise = prior_map_noise_min + fitness_score * prior_map_noise_scale
    // Better match → lower fitness → lower noise → GTSAM trusts it more
    prior_map_noise_min_   = declare_parameter<double>("gtsam.prior_map_noise_min",   0.01);
    prior_map_noise_scale_ = declare_parameter<double>("gtsam.prior_map_noise_scale", 2.0);
    // Initial pose anchor noise — how much GTSAM can deviate from init_pose
    // Set large if your initial position is uncertain; prior map factors will correct it
    init_pose_noise_trans_ = declare_parameter<double>("gtsam.init_pose_noise_trans", 0.5);
    init_pose_noise_rot_   = declare_parameter<double>("gtsam.init_pose_noise_rot",   0.3);

    // ── Build sonar→base transform ────────────────────────
    Eigen::AngleAxisf rx(static_cast<float>(s2b_r   * M_PI/180.0), Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf ry(static_cast<float>(s2b_p   * M_PI/180.0), Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rz(static_cast<float>(s2b_yaw * M_PI/180.0), Eigen::Vector3f::UnitZ());
    sonar2base_.setIdentity();
    sonar2base_.block<3,3>(0,0) = (rz * ry * rx).toRotationMatrix();
    sonar2base_.block<3,1>(0,3) = Eigen::Vector3f(s2b_x, s2b_y, s2b_z);
    base2sonar_ = sonar2base_.inverse();

    prior_map_.reset(new CloudT);

    // ── VGICP setup ───────────────────────────────────────
    local_vgicp_.setNumThreads(4);
    local_vgicp_.setMaxCorrespondenceDistance(local_max_dist_);
    local_vgicp_.setTransformationEpsilon(local_epsilon_);
    local_vgicp_.setMaximumIterations(local_max_iter_);
    local_vgicp_.setResolution(local_resolution_);

    global_vgicp_.setNumThreads(4);
    global_vgicp_.setMaxCorrespondenceDistance(global_max_dist_);
    global_vgicp_.setTransformationEpsilon(global_epsilon_);
    global_vgicp_.setMaximumIterations(global_max_iter_);
    global_vgicp_.setResolution(global_resolution_);
    // ── NDT setup (alternative global matcher) ────────────
    ndt_.setResolution(static_cast<float>(ndt_resolution_));
    ndt_.setStepSize(ndt_step_size_);
    ndt_.setTransformationEpsilon(ndt_epsilon_);
    ndt_.setMaximumIterations(ndt_max_iter_);
    // Targets are set in priorMapCallback once the map arrives

    // ── Filters ───────────────────────────────────────────
    map_filter_.setLeafSize(
        static_cast<float>(local_resolution_),
        static_cast<float>(local_resolution_),
        static_cast<float>(local_resolution_));
    sor_filter_.setMeanK(sor_neighbors_);
    sor_filter_.setStddevMulThresh(sor_stddev_);
    radius_filter_.setRadiusSearch(radius_search_);
    radius_filter_.setMinNeighborsInRadius(radius_min_neighbors_);

    local_map_.reset(new CloudT);

    // ── GTSAM iSAM2 setup ─────────────────────────────────
    gtsam::ISAM2Params isam_params;
    isam_params.relinearizeThreshold = 0.01;
    isam_params.relinearizeSkip      = 1;
    isam2_ = std::make_unique<gtsam::ISAM2>(isam_params);

    // Fixed odometry noise model
    odom_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector6() <<
            odom_noise_rot_,   odom_noise_rot_,   odom_noise_rot_,
            odom_noise_trans_, odom_noise_trans_, odom_noise_trans_
        ).finished());

    // ── ROS interfaces ────────────────────────────────────
    local_map_pub_    = create_publisher<sensor_msgs::msg::PointCloud2>("local_map", 1);
    match_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "match_markers", 10);
    path_pub_        = create_publisher<nav_msgs::msg::Path>("optimized_path", 10);
    vgicp_odom_pub_  = create_publisher<nav_msgs::msg::Odometry>("vgicp_odom", 10);
    full_map_pub_    = create_publisher<sensor_msgs::msg::PointCloud2>("full_map", 1);

    ekf_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 10,
        std::bind(&PriorMapLocalizer::ekfCallback, this, std::placeholders::_1));

    pc_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        pc_topic_, 10,
        std::bind(&PriorMapLocalizer::cloudCallback, this, std::placeholders::_1));

    initial_pose_sub_ = create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 1,
        std::bind(&PriorMapLocalizer::initialPoseCallback, this,
                  std::placeholders::_1));

    // ── Prior map subscription (transient_local = latched) ──
    rclcpp::QoS map_qos(1);
    // map_qos.transient_local().reliable();
    prior_map_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        prior_map_topic, map_qos,
        std::bind(&PriorMapLocalizer::priorMapCallback, this, std::placeholders::_1));

    tf_timer_ = create_wall_timer(
        std::chrono::milliseconds(200),
        [this]() { publishMapToOdomTF(this->now()); });

    publishMapToOdomTF(this->now());

    RCLCPP_INFO(get_logger(),
        "Prior map localizer (GTSAM) ready.\n"
        "  odom_frame: %s  |  prior_map_frame: %s\n"
        "  score_thresh: %.3f  |  match_every: %d KFs",
        odom_frame_.c_str(), prior_map_frame_.c_str(),
        score_thresh_, match_every_n_kf_);
}

// ────────────────────────────────────────────────────────────
//  Callbacks (unchanged from original)
// ────────────────────────────────────────────────────────────
void PriorMapLocalizer::ekfCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg)
{
    Eigen::Quaternionf q(
        msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
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

    // Reset everything
    has_global_match_ = false;
    map_initialized_  = false;
    graph_initialized_= false;
    keyframes_.clear();
    local_map_->clear();
    graph_.resize(0);
    initial_estimates_.clear();
    isam2_ = std::make_unique<gtsam::ISAM2>(gtsam::ISAM2Params());

    publishMapToOdomTF(this->now());
    RCLCPP_INFO(get_logger(), "Initial pose reset from RViz.");
}

void PriorMapLocalizer::priorMapCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    prior_map_.reset(new CloudT);
    pcl::PointCloud<pcl::PointXYZ> tmp;
    pcl::fromROSMsg(*msg, tmp);
    pcl::copyPointCloud(tmp, *prior_map_);

    global_vgicp_.setInputTarget(prior_map_);
    ndt_.setInputTarget(prior_map_);

    has_prior_map_ = true;
    // RCLCPP_INFO(get_logger(), "Prior map received: %zu pts", prior_map_->size());
}

void PriorMapLocalizer::cloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    if (!has_ekf_) {
        RCLCPP_WARN_ONCE(get_logger(), "Waiting for EKF...");
        return;
    }
    if (!has_prior_map_)
        RCLCPP_WARN_ONCE(get_logger(), "Prior map not yet received — running local VGICP only.");

    CloudTPtr raw(new CloudT);
    pcl::fromROSMsg(*msg, *raw);
    CloudTPtr scan = preprocessCloud(raw);
    if (!scan || scan->empty()) return;

    Eigen::Matrix4f ekf_pose;
    { std::lock_guard<std::mutex> lock(ekf_mutex_); ekf_pose = ekf_pose_; }

    if (!buildLocalMap(scan, ekf_pose, rclcpp::Time(msg->header.stamp)))
        return;

    scan_count_++;

    int kf_count = static_cast<int>(keyframes_.size());
    if (has_prior_map_ && kf_count > 0 && kf_count % match_every_n_kf_ == 0)
        matchLocalMapToPrior(rclcpp::Time(msg->header.stamp));

    if (scan_count_ % 5 == 0)
        publishLocalMap(rclcpp::Time(msg->header.stamp));

    if (scan_count_ % 20 == 0)
        publishFullMap(rclcpp::Time(msg->header.stamp));
}

// ────────────────────────────────────────────────────────────
//  Build local map — CHANGED: calls addOdometryFactor
// ────────────────────────────────────────────────────────────
bool PriorMapLocalizer::buildLocalMap(
    const CloudTPtr& scan,
    const Eigen::Matrix4f& ekf_pose,
    const rclcpp::Time& stamp)
{
    if (!map_initialized_) {
        global_pose_   = ekf_pose;
        prev_ekf_pose_ = ekf_pose;
        addKeyframe(scan, global_pose_, stamp);
        updateSubmap();
        map_initialized_ = true;
        RCLCPP_INFO(get_logger(), "Map initialized.");
        return true;
    }

    Eigen::Matrix4f ekf_delta     = prev_ekf_pose_.inverse() * ekf_pose;
    Eigen::Matrix4f initial_guess = global_pose_ * ekf_delta;

    float delta_dist  = ekf_delta.block<3,1>(0,3).norm();
    Eigen::AngleAxisf aa(Eigen::Matrix3f(ekf_delta.block<3,3>(0,0)));
    float delta_angle = std::abs(aa.angle()) * 180.0f / M_PI;

    // if (delta_dist > 2.0f || delta_angle > 45.0f) {
    //     RCLCPP_WARN(get_logger(), "Large EKF delta — using last pose.");
    //     initial_guess = global_pose_;
    // }

    local_vgicp_.setInputTarget(local_map_);
    local_vgicp_.setInputSource(scan);
    CloudT aligned;
    local_vgicp_.align(aligned, initial_guess);

    if (!local_vgicp_.hasConverged()) {
        // RCLCPP_WARN(get_logger(), "Local VGICP failed — using EKF.");
        global_pose_   = initial_guess;
        prev_ekf_pose_ = ekf_pose;
        return true;
    }

    double score   = local_vgicp_.getFitnessScore();
    global_pose_   = local_vgicp_.getFinalTransformation();
    prev_ekf_pose_ = ekf_pose;

    // Publish local VGICP odometry in odom_frame
    {
        Eigen::Quaternionf q(global_pose_.block<3,3>(0,0));
        nav_msgs::msg::Odometry odom;
        odom.header.stamp    = stamp;
        odom.header.frame_id = odom_frame_;
        odom.child_frame_id  = base_frame_;
        odom.pose.pose.position.x    = global_pose_(0,3);
        odom.pose.pose.position.y    = global_pose_(1,3);
        odom.pose.pose.position.z    = global_pose_(2,3);
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();
        vgicp_odom_pub_->publish(odom);
    }

    if (shouldAddKeyframe(global_pose_)) {
        int prev_id = static_cast<int>(keyframes_.size()) - 1;
        Eigen::Matrix4f delta_from_prev =
            keyframes_[prev_id].pose.inverse() * global_pose_;

        addKeyframe(scan, global_pose_, stamp);

        // ── NEW: add odometry factor between consecutive keyframes ──
        addOdometryFactor(prev_id,
                          static_cast<int>(keyframes_.size()) - 1,
                          delta_from_prev,
                          score);
        updateSubmap();
    }

    return true;
}

// ────────────────────────────────────────────────────────────
//  Keyframe management — CHANGED: addKeyframe also adds GTSAM node
// ────────────────────────────────────────────────────────────
bool PriorMapLocalizer::shouldAddKeyframe(const Eigen::Matrix4f& pose)
{
    if (keyframes_.empty()) return true;
    Eigen::Matrix4f d = keyframes_.back().pose.inverse() * pose;
    float dist  = d.block<3,1>(0,3).norm();
    Eigen::AngleAxisf aa(Eigen::Matrix3f(d.block<3,3>(0,0)));
    float angle = std::abs(aa.angle()) * 180.0f / M_PI;
    return dist > static_cast<float>(kf_dist_thresh_) ||
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

    // ── NEW: Add node to GTSAM graph ─────────────────────
    // The pose here is in the odom frame.
    // We convert to map frame using map_T_odom_ so GTSAM
    // works in the same frame as the prior map factors.
    Eigen::Matrix4f pose_in_map = map_T_odom_ * pose;
    gtsam::Pose3 gtsam_pose = eigenToGtsam(pose_in_map);
    initial_estimates_.insert(kf.id, gtsam_pose);

    if (kf.id == 0) {
        // ── First keyframe: add a prior to anchor the graph ──
        // Use a loose prior — the prior MAP factors will provide
        // the real absolute constraint. This just prevents the
        // graph from being underconstrained.
        auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector6() <<
                init_pose_noise_rot_,   init_pose_noise_rot_,   init_pose_noise_rot_,
                init_pose_noise_trans_, init_pose_noise_trans_, init_pose_noise_trans_
            ).finished());
        graph_.add(gtsam::PriorFactor<gtsam::Pose3>(
            0, gtsam_pose, prior_noise));
        graph_initialized_ = true;
    }

    RCLCPP_INFO(get_logger(),
        "KF %d added | pos=[%.2f, %.2f, %.2f]",
        kf.id, pose(0,3), pose(1,3), pose(2,3));
}

void PriorMapLocalizer::updateSubmap()
{
    local_map_->clear();
    int start = std::max(0, static_cast<int>(keyframes_.size()) - submap_size_);
    for (int i = start; i < static_cast<int>(keyframes_.size()); ++i) {
        CloudT tmp;
        pcl::transformPointCloud(*keyframes_[i].cloud, tmp, keyframes_[i].pose);
        *local_map_ += tmp;
    }
    voxelFilter(local_map_, static_cast<float>(submap_voxel_res_));
}

// ────────────────────────────────────────────────────────────
//  GTSAM factors — NEW functions
// ────────────────────────────────────────────────────────────
void PriorMapLocalizer::addOdometryFactor(
    int from_id, int to_id,
    const Eigen::Matrix4f& delta,
    double fitness_score)
{
    // Scale odometry noise with fitness score
    // Better local match → tighter constraint
    double scale = 1.0 + fitness_score * 5.0;
    auto noise = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector6() <<
            odom_noise_rot_   * scale, odom_noise_rot_   * scale,
            odom_noise_rot_   * scale,
            odom_noise_trans_ * scale, odom_noise_trans_ * scale,
            odom_noise_trans_ * scale
        ).finished());

    graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
        from_id, to_id, eigenToGtsam(delta), noise));

    RCLCPP_DEBUG(get_logger(),
        "Odom factor %d→%d | score=%.4f noise_scale=%.2f",
        from_id, to_id, fitness_score, scale);
}

void PriorMapLocalizer::addPriorMapFactor(
    int kf_id,
    const Eigen::Matrix4f& abs_pose_in_map,
    double fitness_score)
{
    // ── Scale noise with fitness score ────────────────────
    // Low score = good match = tight noise = GTSAM trusts it strongly
    // High score = poor match = loose noise = GTSAM trusts it less
    double noise_val = prior_map_noise_min_ +
                       fitness_score * prior_map_noise_scale_;
    double noise_rot = noise_val * 0.5;

    auto noise = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector6() <<
            noise_rot, noise_rot, noise_rot,
            noise_val, noise_val, noise_val
        ).finished());

    graph_.add(gtsam::PriorFactor<gtsam::Pose3>(
        kf_id, eigenToGtsam(abs_pose_in_map), noise));

    RCLCPP_INFO(get_logger(),
        "Prior map factor on KF %d | score=%.4f noise=%.4f",
        kf_id, fitness_score, noise_val);
}

void PriorMapLocalizer::optimizeGraph()
{
    if (!graph_initialized_ || initial_estimates_.empty()) return;

    try {
        // Feed new factors and initial estimates into iSAM2
        isam2_->update(graph_, initial_estimates_);
        isam2_->update();  // extra pass for better convergence

        // Clear — iSAM2 remembers history internally
        graph_.resize(0);
        initial_estimates_.clear();

        RCLCPP_INFO(get_logger(),
            "Graph optimized. %zu keyframes corrected.", keyframes_.size());

    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "GTSAM optimization failed: %s", e.what());
    }
}

void PriorMapLocalizer::updateKeyframePosesFromGraph()
{
    try {
        gtsam::Values optimized = isam2_->calculateEstimate();

        int latest_id = static_cast<int>(keyframes_.size()) - 1;
        if (keyframes_.empty() || !optimized.exists(latest_id)) return;

        // Derive map_T_odom_ from the latest keyframe
        // (it has the direct prior map factor → GTSAM corrects it most)
        Eigen::Matrix4f kfN_in_map  = gtsamToEigen(optimized.at<gtsam::Pose3>(latest_id));
        Eigen::Matrix4f kfN_in_odom = keyframes_[latest_id].pose;
        map_T_odom_ = kfN_in_map * kfN_in_odom.inverse();

    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(),
            "Failed to extract optimized poses: %s", e.what());
    }
}

// ────────────────────────────────────────────────────────────
//  Match local map to prior — CHANGED: uses addPriorMapFactor
// ────────────────────────────────────────────────────────────
void PriorMapLocalizer::matchLocalMapToPrior(const rclcpp::Time& stamp)
{
    if (local_map_->empty()) return;

    // Transform local map into map frame for comparison with prior
    CloudTPtr local_in_map(new CloudT);
    pcl::transformPointCloud(*local_map_, *local_in_map, map_T_odom_);

    CloudT aligned;
    bool   hasConverged = false;
    double score        = 0.0;
    Eigen::Matrix4f T_correction = Eigen::Matrix4f::Identity();

    if (use_ndt_) {
        ndt_.setInputSource(local_in_map);
        ndt_.align(aligned, Eigen::Matrix4f::Identity());
        hasConverged = ndt_.hasConverged();
        score        = ndt_.getFitnessScore();
        T_correction = ndt_.getFinalTransformation();
    } else {
        global_vgicp_.setInputSource(local_in_map);
        global_vgicp_.align(aligned, Eigen::Matrix4f::Identity());
        hasConverged = global_vgicp_.hasConverged();
        score        = global_vgicp_.getFitnessScore();
        T_correction = global_vgicp_.getFinalTransformation();
    }

    const char* matcher_name = use_ndt_ ? "NDT" : "VGICP";
    if (!hasConverged) {
        RCLCPP_WARN(get_logger(), "Global %s did not converge.", matcher_name);
        publishMapToOdomTF(stamp);
        return;
    }

    RCLCPP_INFO(get_logger(),
        "Global %s match: score=%.4f (thresh=%.4f)", matcher_name, score, score_thresh_);

    if (score > score_thresh_) {
        RCLCPP_WARN(get_logger(), "Poor %s match (%.4f) — skipping.", matcher_name, score);
        publishMapToOdomTF(stamp);
        return;
    }

    // ── Compute absolute pose of current keyframe in MAP frame ──
    // Correction × current submap pose in map frame
    Eigen::Matrix4f abs_pose_in_map = T_correction * map_T_odom_ * global_pose_;

    // ── Add prior map factor for the latest keyframe ─────
    int latest_kf_id = static_cast<int>(keyframes_.size()) - 1;
    addPriorMapFactor(latest_kf_id, abs_pose_in_map, score);

    // ── Optimize the full graph ───────────────────────────
    optimizeGraph();

    // ── Update map_T_odom_ from GTSAM result ─────────────
    updateKeyframePosesFromGraph();

    has_global_match_ = true;
    publishMatchMarker(abs_pose_in_map, stamp, score);
    publishOptimizedPath(stamp);
    publishMapToOdomTF(stamp);

    RCLCPP_INFO(get_logger(),
        "✓ GTSAM correction applied. pos=[%.2f, %.2f, %.2f]",
        global_pose_(0,3), global_pose_(1,3), global_pose_(2,3));
}

// ────────────────────────────────────────────────────────────
//  GTSAM ↔ Eigen conversions
// ────────────────────────────────────────────────────────────
gtsam::Pose3 PriorMapLocalizer::eigenToGtsam(const Eigen::Matrix4f& m)
{
    Eigen::Matrix3d R = m.block<3,3>(0,0).cast<double>();
    Eigen::Vector3d t = m.block<3,1>(0,3).cast<double>();
    return gtsam::Pose3(gtsam::Rot3(R), gtsam::Point3(t));
}

Eigen::Matrix4f PriorMapLocalizer::gtsamToEigen(const gtsam::Pose3& p)
{
    Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
    m.block<3,3>(0,0) = p.rotation().matrix().cast<float>();
    m.block<3,1>(0,3) = p.translation().cast<float>();
    return m;
}

// ────────────────────────────────────────────────────────────
//  Preprocessing (unchanged from original)
// ────────────────────────────────────────────────────────────
CloudTPtr PriorMapLocalizer::preprocessCloud(const CloudTPtr& raw)
{
    if (!raw || raw->empty()) return nullptr;

    CloudTPtr filtered(new CloudT);
    if (use_intensity_filter_) {
        filtered->reserve(raw->size());
        for (const auto& pt : *raw)
            if (std::isfinite(pt.x) && pt.intensity > min_intensity_)
                filtered->push_back(pt);
    } else {
        for (const auto& pt : *raw)
            if (std::isfinite(pt.x))
                filtered->push_back(pt);
    }
    if (filtered->empty()) return nullptr;

    CloudTPtr in_base(new CloudT);
    pcl::transformPointCloud(*filtered, *in_base, base2sonar_);

    CloudTPtr sor_out(new CloudT);
    if (use_sor_ && in_base->size() > static_cast<size_t>(sor_neighbors_)) {
        sor_filter_.setInputCloud(in_base);
        sor_filter_.filter(*sor_out);
    } else { sor_out = in_base; }

    CloudTPtr clean(new CloudT);
    if (use_radius_ && sor_out->size() > 10) {
        radius_filter_.setInputCloud(sor_out);
        radius_filter_.filter(*clean);
    } else { clean = sor_out; }

    if (clean->empty()) return nullptr;

    CloudTPtr dbscan_out = dbscanFilter(clean);
    return (dbscan_out && !dbscan_out->empty()) ? dbscan_out : nullptr;
}

void PriorMapLocalizer::voxelFilter(CloudTPtr& cloud, float leaf)
{
    if (!cloud || cloud->empty()) return;
    pcl::VoxelGrid<PointT> vg;
    vg.setLeafSize(leaf, leaf, leaf);
    vg.setInputCloud(cloud);
    CloudTPtr out(new CloudT);
    vg.filter(*out);
    cloud = out;
}

// ────────────────────────────────────────────────────────────
//  DBSCAN cluster filter — keeps the N largest clusters
// ────────────────────────────────────────────────────────────
CloudTPtr PriorMapLocalizer::dbscanFilter(const CloudTPtr& cloud)
{
    if (!use_dbscan_ || !cloud || cloud->empty()) return cloud;

    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusters;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(static_cast<float>(dbscan_tolerance_));
    ec.setMinClusterSize(dbscan_min_pts_);
    ec.setMaxClusterSize(static_cast<int>(cloud->size()));
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusters);

    if (clusters.empty()) {
        RCLCPP_WARN(get_logger(), "DBSCAN: no clusters found, returning original cloud.");
        return cloud;
    }

    // Sort largest first
    std::sort(clusters.begin(), clusters.end(),
        [](const pcl::PointIndices& a, const pcl::PointIndices& b) {
            return a.indices.size() > b.indices.size();
        });

    CloudTPtr result(new CloudT);
    result->header = cloud->header;
    int n = std::min(dbscan_keep_clusters_, static_cast<int>(clusters.size()));
    for (int i = 0; i < n; ++i)
        for (int idx : clusters[i].indices)
            result->push_back((*cloud)[idx]);

    RCLCPP_INFO(get_logger(), "DBSCAN: %zu clusters, kept %d (%zu → %zu pts)",
        clusters.size(), n, cloud->size(), result->size());

    return result;
}

// ────────────────────────────────────────────────────────────
//  Publishers (unchanged from original)
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


void PriorMapLocalizer::publishMatchMarker(
    const Eigen::Matrix4f& pose,
    const rclcpp::Time& stamp,
    double score)
{
    visualization_msgs::msg::MarkerArray ma;
    visualization_msgs::msg::Marker m;
    m.header.frame_id = prior_map_frame_;
    m.header.stamp    = stamp;
    m.ns = "global_match"; m.id = 0;
    m.type   = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = pose(0,3);
    m.pose.position.y = pose(1,3);
    m.pose.position.z = pose(2,3);
    m.pose.orientation.w = 1.0;
    m.scale.x = m.scale.y = m.scale.z = 0.5;
    double t = std::min(score / score_thresh_, 1.0);
    m.color.r = static_cast<float>(t);
    m.color.g = 1.0f; m.color.b = 0.0f; m.color.a = 1.0f;
    m.lifetime = rclcpp::Duration::from_seconds(0);
    ma.markers.push_back(m);
    match_marker_pub_->publish(ma);
}

void PriorMapLocalizer::publishFullMap(const rclcpp::Time& stamp)
{
    if (keyframes_.empty()) return;

    CloudTPtr full(new CloudT);
    for (const auto& kf : keyframes_) {
        CloudT transformed;
        pcl::transformPointCloud(*kf.cloud, transformed, kf.pose);
        *full += transformed;
    }

    voxelFilter(full, static_cast<float>(local_resolution_));

    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*full, msg);
    msg.header.frame_id = odom_frame_;
    msg.header.stamp    = stamp;
    full_map_pub_->publish(msg);
}

void PriorMapLocalizer::publishOptimizedPath(const rclcpp::Time& stamp)
{
    nav_msgs::msg::Path path;
    path.header.frame_id = prior_map_frame_;
    path.header.stamp    = stamp;

    for (const auto& kf : keyframes_) {
        // Convert keyframe pose from ekf_odom to map frame
        Eigen::Matrix4f pose_in_map = map_T_odom_ * kf.pose;
        Eigen::Quaternionf q(pose_in_map.block<3,3>(0,0));

        geometry_msgs::msg::PoseStamped ps;
        ps.header = path.header;
        ps.pose.position.x    = pose_in_map(0,3);
        ps.pose.position.y    = pose_in_map(1,3);
        ps.pose.position.z    = pose_in_map(2,3);
        ps.pose.orientation.x = q.x();
        ps.pose.orientation.y = q.y();
        ps.pose.orientation.z = q.z();
        ps.pose.orientation.w = q.w();
        path.poses.push_back(ps);
    }

    path_pub_->publish(path);
}

void PriorMapLocalizer::publishMapToOdomTF(const rclcpp::Time& stamp)
{
    Eigen::Quaternionf q(map_T_odom_.block<3,3>(0,0));
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp    = stamp;
    t.header.frame_id = prior_map_frame_;
    t.child_frame_id  = odom_frame_;
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
            "Fatal: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}