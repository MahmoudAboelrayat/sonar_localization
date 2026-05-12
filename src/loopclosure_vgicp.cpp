#include <memory>
#include <string>
#include <mutex>
#include <vector>
#include <cmath>
#include <atomic>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/header.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <fast_gicp/gicp/fast_vgicp.hpp>
#include <pcl/registration/ndt.h>
#include <Eigen/Geometry>

// --- GTSAM Headers ---
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

using gtsam::symbol_shorthand::X;

struct Keyframe {
    Eigen::Matrix4f pose;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    int id;
    float ekf_z{0.0f};   // EKF depth at keyframe time — never overwritten by GTSAM
};

class GicpOdomNode : public rclcpp::Node
{
public:
    GicpOdomNode(const rclcpp::NodeOptions & options) : Node("vgicp_odom_node", options)
    {
        // --- Parameters ---
        std::string odom_pub_topic    = this->declare_parameter<std::string>("topics.odom_pub",           "vgicp_odom");
        std::string pc_sub_topic      = this->declare_parameter<std::string>("topics.pc_sub",             "/sonar/point_cloud_noisy");
        std::string deadreckoning_sub = this->declare_parameter<std::string>("topics.deadreckoining_sub", "/odometry/filtered");
        std::string map_pub_topic     = this->declare_parameter<std::string>("topics.map_pub",            "vgicp_global_map");

        double map_res        = this->declare_parameter<double>("tuning.map_res",          0.1);
        int    vgicp_threads  = this->declare_parameter<int>   ("tuning.vgicp_threads",    4);
        double vgicp_epsilon  = this->declare_parameter<double>("tuning.vgicp_epsilon",    1e-4);
        double vgicp_max_dist = this->declare_parameter<double>("tuning.vgicp_max_dist",   1.5);
        int    vgicp_max_iter = this->declare_parameter<int>   ("tuning.vgicp_max_iter",   100);
        double vgicp_res      = this->declare_parameter<double>("tuning.vgicp_resolution", 0.25);
        max_lost_frames = this->declare_parameter<int>("tuning.max_lost_frames", 40);
        min_ekf_msgs_       = this->declare_parameter<int>("tuning.min_ekf_msgs",    20);


        double lc_vgicp_epsilon  = this->declare_parameter<double>("loop_closure.vgicp_epsilon",    1e-4);
        double lc_vgicp_max_dist = this->declare_parameter<double>("loop_closure.vgicp_max_dist",   1.5);
        int    lc_vgicp_max_iter = this->declare_parameter<int>   ("loop_closure.vgicp_max_iter",   100);
        double lc_vgicp_res      = this->declare_parameter<double>("loop_closure.vgicp_resolution", 0.25);

        lc_use_ndt_      = this->declare_parameter<bool>  ("loop_closure.use_ndt",         false);
        double lc_ndt_res  = this->declare_parameter<double>("loop_closure.ndt_resolution",  2.0);
        double lc_ndt_step = this->declare_parameter<double>("loop_closure.ndt_step_size",   0.5);
        double lc_ndt_eps  = this->declare_parameter<double>("loop_closure.ndt_epsilon",     0.01);
        int    lc_ndt_iter = this->declare_parameter<int>  ("loop_closure.ndt_max_iter",     50);

        odom_frame_    = this->declare_parameter<std::string>("frames.odom_frame", "odom");
        base_frame_    = this->declare_parameter<std::string>("frames.base_frame", "sam_auv_v1/base_link");
        min_intensity = static_cast<float>(this->declare_parameter<double>("tuning.min_intensity", 0.0));
        is_ned_        = this->declare_parameter<bool>("is_ned", false);
        ekf_z_         = this->declare_parameter<bool>("ekf_z", false);
        loop_ekf_z_    = this->declare_parameter<bool>("loop_ekf_z", false);
        ekf_max_age_   = this->declare_parameter<double>("ekf_max_age", 0.1);

        submap_size_     = this->declare_parameter<int>   ("keyframe.submap_size",  20);
        kf_dist_thresh_  = this->declare_parameter<double>("keyframe.dist_thresh",  0.5);
        kf_angle_thresh_ = this->declare_parameter<double>("keyframe.angle_thresh", 10.0);

        lc_search_radius_ = this->declare_parameter<double>("loop_closure.search_radius", 10.0);
        lc_fitness_score_ = this->declare_parameter<double>("loop_closure.fitness_score", 0.3);
        lc_history_gap_   = this->declare_parameter<int>   ("loop_closure.history_gap",   10);
        lc_submap_size_   = this->declare_parameter<int>   ("loop_closure.submap_size",   7);

        // GICP sanity check thresholds
        gicp_max_correction_dist_  = this->declare_parameter<double>("tuning.gicp_max_correction_dist",  1.0);
        gicp_max_correction_angle_ = this->declare_parameter<double>("tuning.gicp_max_correction_angle", 15.0);

        // Extrinsics: translation + RPY in degrees (sonar -> base)
        double s2b_t_x   = this->declare_parameter<double>("tf.sonar2base_x",      -0.545);
        double s2b_t_y   = this->declare_parameter<double>("tf.sonar2base_y",       0.000);
        double s2b_t_z   = this->declare_parameter<double>("tf.sonar2base_z",      -0.404);
        double s2b_roll  = this->declare_parameter<double>("tf.sonar2base_roll",    0.0);
        double s2b_pitch = this->declare_parameter<double>("tf.sonar2base_pitch",  -30.0);
        double s2b_yaw   = this->declare_parameter<double>("tf.sonar2base_yaw",     0.0);

        Eigen::Quaternionf rotation_sb;
        rotation_sb = Eigen::AngleAxisf(static_cast<float>(s2b_yaw   * M_PI / 180.0), Eigen::Vector3f::UnitZ())
                    * Eigen::AngleAxisf(static_cast<float>(s2b_pitch  * M_PI / 180.0), Eigen::Vector3f::UnitY())
                    * Eigen::AngleAxisf(static_cast<float>(s2b_roll   * M_PI / 180.0), Eigen::Vector3f::UnitX());

        sonar2base_ = Eigen::Matrix4f::Identity();
        sonar2base_.block<3,3>(0,0) = rotation_sb.toRotationMatrix();
        sonar2base_.block<3,1>(0,3) = Eigen::Vector3f(
            static_cast<float>(s2b_t_x),
            static_cast<float>(s2b_t_y),
            static_cast<float>(s2b_t_z));
        base2sonar_ = sonar2base_.inverse();

        ned_transform_ << 0, -1,  0, 0,
                          1,  0,  0, 0,
                          0,  0, -1, 0,
                          0,  0,  0, 1;

        RCLCPP_INFO(get_logger(),
            "Sonar->Base extrinsics: t=[%.3f, %.3f, %.3f] rpy=[%.1f, %.1f, %.1f] deg",
            s2b_t_x, s2b_t_y, s2b_t_z, s2b_roll, s2b_pitch, s2b_yaw);

        // --- Publishers ---
        odom_pub_       = this->create_publisher<nav_msgs::msg::Odometry>       (odom_pub_topic,    10);
        global_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2> (map_pub_topic,      1);
        full_map_pub_   = this->create_publisher<sensor_msgs::msg::PointCloud2> ("vgicp_full_map",   1);
        path_pub_       = this->create_publisher<nav_msgs::msg::Path>           ("vgicp_path",      10);
        lc_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
                        "vgicp/loop_closure_constraints", 1);
        // --- Subscriptions ---
        auto pc_cb_group  = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto ekf_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        auto pc_sub_opt  = rclcpp::SubscriptionOptions();
        pc_sub_opt.callback_group = pc_cb_group;

        auto ekf_sub_opt = rclcpp::SubscriptionOptions();
        ekf_sub_opt.callback_group = ekf_cb_group;

        pc_sub_  = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            pc_sub_topic, 10,
            std::bind(&GicpOdomNode::pointCloudCallback, this, std::placeholders::_1),
            pc_sub_opt);

        ekf_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            deadreckoning_sub, 10,
            std::bind(&GicpOdomNode::ekfCallback, this, std::placeholders::_1),
            ekf_sub_opt);


        // Loop closure in a plain std::thread — same pattern as LIO-SAM
        loop_closure_thread_ = std::thread(&GicpOdomNode::loopClosureThread, this);

        // --- State init ---
        global_pose_     = Eigen::Matrix4f::Identity();
        latest_ekf_pose_ = Eigen::Matrix4f::Identity();
        prev_ekf_pose_   = Eigen::Matrix4f::Identity();

        local_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        map_filter_.setLeafSize(map_res, map_res, map_res);

        // VGICP — odometry (scan-to-local-map)
        vgicp_.setNumThreads(vgicp_threads);
        vgicp_.setTransformationEpsilon(vgicp_epsilon);
        vgicp_.setMaxCorrespondenceDistance(vgicp_max_dist);
        vgicp_.setMaximumIterations(vgicp_max_iter);
        vgicp_.setResolution(vgicp_res);

        // VGICP — loop closure (world-frame clouds, wider search)
        vgicp_lc_.setNumThreads(vgicp_threads);
        vgicp_lc_.setMaxCorrespondenceDistance(lc_vgicp_max_dist);
        vgicp_lc_.setMaximumIterations(lc_vgicp_max_iter);
        vgicp_lc_.setTransformationEpsilon(lc_vgicp_epsilon);
        vgicp_lc_.setResolution(lc_vgicp_res);

        // NDT — loop closure (alternative to VGICP, better for large initial offsets)
        ndt_lc_.setResolution(static_cast<float>(lc_ndt_res));
        ndt_lc_.setStepSize(lc_ndt_step);
        ndt_lc_.setTransformationEpsilon(lc_ndt_eps);
        ndt_lc_.setMaximumIterations(lc_ndt_iter);

        // setup outlier removal (optional)
        filter_outliers = this->declare_parameter<bool>("outlier_removal.filter_outliers", true);
        int num_neighbors = this->declare_parameter<int>("outlier_removal.num_neighbors", 20);
        sor_.setMeanK(num_neighbors);
        double stddev_mul_thresh = this->declare_parameter<double>("outlier_removal.stddev_mul_thresh", 1.5);
        sor_.setStddevMulThresh(stddev_mul_thresh);

        // setup radius outlier removal (optional)
        filter_radius_outliers_ = this->declare_parameter<bool>("radius_outlier_removal.filter_radius_outliers", false);
        double ror_radius = this->declare_parameter<double>("radius_outlier_removal.search_radius", 0.5);
        int ror_min_neighbors = this->declare_parameter<int>("radius_outlier_removal.min_neighbors", 5);
        ror_.setRadiusSearch(ror_radius);
        ror_.setMinNeighborsInRadius(ror_min_neighbors);

        // setup intensity filter (optional)
        filter_intensity = this->declare_parameter<bool>("intensity_filter.filter_intensity", false);
        min_intensity = this->declare_parameter<double>("intensity_filter.min_intensity", 0.0);

        // odom noise sigmas [roll, pitch, yaw, x, y, z]
        odom_noise_roll_  = this->declare_parameter<double>("gtsam.odom_noise_roll",  0.1);
        odom_noise_pitch_ = this->declare_parameter<double>("gtsam.odom_noise_pitch", 0.1);
        odom_noise_yaw_   = this->declare_parameter<double>("gtsam.odom_noise_yaw",   0.3);
        odom_noise_x_     = this->declare_parameter<double>("gtsam.odom_noise_x",     0.5);
        odom_noise_y_     = this->declare_parameter<double>("gtsam.odom_noise_y",     0.5);
        odom_noise_z_     = this->declare_parameter<double>("gtsam.odom_noise_z",     0.3);

        lc_noise_roll_    = this->declare_parameter<double>("gtsam.lc_noise_roll",    0.01);
        lc_noise_pitch_   = this->declare_parameter<double>("gtsam.lc_noise_pitch",   0.01);
        lc_noise_yaw_     = this->declare_parameter<double>("gtsam.lc_noise_yaw",     0.01);
        lc_noise_x_       = this->declare_parameter<double>("gtsam.lc_noise_x",       0.05);
        lc_noise_y_       = this->declare_parameter<double>("gtsam.lc_noise_y",       0.05);
        lc_noise_z_       = this->declare_parameter<double>("gtsam.lc_noise_z",       0.05);
        lc_huber_k_       = this->declare_parameter<double>("gtsam.lc_huber_k",       1.0);

        initGTSAM();

        RCLCPP_INFO(get_logger(), "VGICP SLAM Node initialised.");
    }

    ~GicpOdomNode()
    {
        if (loop_closure_thread_.joinable())
            loop_closure_thread_.join();
    }

private:
    // ── GTSAM initialisation ──────────────────────────────────────────────────
    // Always call while holding gtsam_mutex_ (except from constructor)
    void initGTSAM()
    {
        gtsam::ISAM2Params params;
        params.relinearizeThreshold = 0.1;
        params.relinearizeSkip      = 1;
        isam_ = std::make_unique<gtsam::ISAM2>(params);

        gtSAMgraph_.resize(0);
        initialEstimates_.clear();

        priorNoise_ = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-1, 1e-1, 1e-1).finished());

        // Loose enough that loop closure can pull the graph
        odomNoise_ = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << odom_noise_roll_, odom_noise_pitch_, odom_noise_yaw_,
                                 odom_noise_x_,    odom_noise_y_,     odom_noise_z_).finished());

        // Fixed tight LC noise + Huber robust kernel
        // Much tighter than odomNoise_ so LC corrections propagate strongly
        auto lc_base = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << lc_noise_roll_, lc_noise_pitch_, lc_noise_yaw_,
                                 lc_noise_x_,    lc_noise_y_,     lc_noise_z_).finished());
        robustLoopNoise_ = gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Huber::Create(lc_huber_k_), lc_base);
    }

    // ── Helpers ───────────────────────────────────────────────────────────────
    gtsam::Pose3 matrix2Pose3(const Eigen::Matrix4f & m)
    {
        return gtsam::Pose3(
            gtsam::Rot3(m.block<3,3>(0,0).cast<double>()),
            gtsam::Point3(m.block<3,1>(0,3).cast<double>()));
    }

    Eigen::Matrix4f pose32Matrix(const gtsam::Pose3 & p)
    {
        Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
        m.block<3,3>(0,0) = p.rotation().matrix().cast<float>();
        m.block<3,1>(0,3) = p.translation().cast<float>();
        return m;
    }

    // ── Keyframe management ───────────────────────────────────────────────────
    void AddKeyFrame(const Eigen::Matrix4f & current_pose,
                     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                     float ekf_z = 0.0f)
    {
        // Check keyframe threshold — quick check without GTSAM lock
        {
            std::lock_guard<std::mutex> kf_lock(kf_mutex_);
            if (!keyframes_.empty()) {
                Eigen::Matrix4f delta = keyframes_.back().pose.inverse() * current_pose;
                float dist  = delta.block<3,1>(0,3).norm();
                float angle = Eigen::AngleAxisf(Eigen::Matrix3f(delta.block<3,3>(0,0))).angle()
                              * 180.0f / M_PI;
                if (dist <= kf_dist_thresh_ && angle <= kf_angle_thresh_) return;
            }
        }

        // Always lock gtsam_mutex_ BEFORE kf_mutex_
        std::lock_guard<std::mutex> gtsam_lock(gtsam_mutex_);
        std::lock_guard<std::mutex> kf_lock   (kf_mutex_);

        int current_id = static_cast<int>(keyframes_.size());
        gtsam::Pose3 current_gtsam_pose = matrix2Pose3(current_pose);

        if (current_id == 0) {
            gtSAMgraph_.add(gtsam::PriorFactor<gtsam::Pose3>(
                X(0), current_gtsam_pose, priorNoise_));
            initialEstimates_.insert(X(0), current_gtsam_pose);
        } else {
            gtsam::Pose3 prev_gtsam = matrix2Pose3(keyframes_.back().pose);
            gtsam::Pose3 relative   = prev_gtsam.between(current_gtsam_pose);
            gtSAMgraph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
                X(current_id - 1), X(current_id), relative, odomNoise_));
            initialEstimates_.insert(X(current_id), current_gtsam_pose);
        }

        isam_->update(gtSAMgraph_, initialEstimates_);
        isam_->update();
        gtSAMgraph_.resize(0);
        initialEstimates_.clear();

        gtsam::Pose3    optimised_pose = isam_->calculateEstimate<gtsam::Pose3>(X(current_id));
        Eigen::Matrix4f optimised_mat  = pose32Matrix(optimised_pose);
        if (ekf_z_ && loop_ekf_z_) optimised_mat(2, 3) = ekf_z;

        {
            std::lock_guard<std::mutex> pose_lock(pose_mutex_);
            global_pose_ = optimised_mat;
        }

        Keyframe kf;
        kf.pose  = optimised_mat;
        kf.cloud = cloud;
        kf.id    = current_id;
        kf.ekf_z = ekf_z;
        keyframes_.push_back(kf);

        // ── LIO-SAM correctPoses() equivalent ────────────────────────────────
        // Fires on the next keyframe after loop closure is detected
        // Corrects ALL poses from GTSAM, rebuilds map and path
        publishFullMap();  // rebuild full map with corrected poses

        if (loop_closure_detected_) {
            try {
                int num_poses = static_cast<int>(keyframes_.size());
                for (int i = 0; i < num_poses; ++i) {
                    keyframes_[i].pose = pose32Matrix(
                        isam_->calculateEstimate<gtsam::Pose3>(X(i)));
                    if (ekf_z_ && loop_ekf_z_) keyframes_[i].pose(2, 3) = keyframes_[i].ekf_z;
                }
                {
                    std::lock_guard<std::mutex> pose_lock(pose_mutex_);
                    global_pose_ = keyframes_.back().pose;
                }
                publishPath();     // snap path to corrected trajectory
                RCLCPP_WARN(get_logger(), "Poses corrected after loop closure. %d keyframes updated.",
                            num_poses);
            } catch (const std::exception & e) {
                RCLCPP_ERROR(get_logger(), "GTSAM pose correction failed: %s", e.what());
            }
            loop_closure_detected_ = false;
        }

        updateSubmap();
        publishPath();   // grow path normally every keyframe

        RCLCPP_INFO(get_logger(), "Keyframe %d added.", current_id);
    }

    // ── Loop closure thread ───────────────────────────────────────────────────
    void loopClosureThread()
    {
        while (rclcpp::ok()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            performLoopClosure();
        }
    }

    // ── Loop closure — LIO-SAM style ──────────────────────────────────────────
    void performLoopClosure()
    {
        // ── 1. Snapshot + candidate search ───────────────────────────────────
        int     latest_id;
        int     closest_id = -1;
        int     generation_before_gicp;

        pcl::PointCloud<pcl::PointXYZ>::Ptr latest_cloud_world (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr history_cloud_world(new pcl::PointCloud<pcl::PointXYZ>);
        Eigen::Matrix4f latest_pose_world;
        Eigen::Matrix4f history_pose_world;

        {
            std::lock_guard<std::mutex> kf_lock(kf_mutex_);
            if (static_cast<int>(keyframes_.size()) < lc_history_gap_ + 5) return;

            latest_id              = static_cast<int>(keyframes_.size()) - 1;
            generation_before_gicp = slam_generation_.load();
            latest_pose_world      = keyframes_[latest_id].pose;

            // Find nearest historical keyframe within search radius
            float min_dist = static_cast<float>(lc_search_radius_);
            for (int i = 0; i < latest_id - lc_history_gap_; ++i) {
                float dist = (latest_pose_world.block<3,1>(0,3)
                            - keyframes_[i].pose.block<3,1>(0,3)).norm();
                if (dist < min_dist) { min_dist = dist; closest_id = i; }
            }

            if (closest_id == -1) {
                RCLCPP_DEBUG(get_logger(),
                    "No loop candidate within %.1fm of kf %d", lc_search_radius_, latest_id);
                return;
            }

            history_pose_world = keyframes_[closest_id].pose;

            // Build latest cloud in WORLD frame
            pcl::transformPointCloud(*keyframes_[latest_id].cloud,
                                     *latest_cloud_world, latest_pose_world);

            // Build history SUBMAP in WORLD frame (±lc_submap_size_ keyframes)
            for (int j = -lc_submap_size_; j <= lc_submap_size_; ++j) {
                int idx = closest_id + j;
                if (idx < 0 || idx >= latest_id) continue;
                pcl::PointCloud<pcl::PointXYZ> transformed;
                pcl::transformPointCloud(*keyframes_[idx].cloud, transformed, keyframes_[idx].pose);
                *history_cloud_world += transformed;
            }
        }

        RCLCPP_INFO(get_logger(), "Loop candidate: kf %d -> %d", latest_id, closest_id);

        // ── 2. Downsample history submap ──────────────────────────────────────
        pcl::PointCloud<pcl::PointXYZ>::Ptr history_ds(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> ds_filter;
        ds_filter.setLeafSize(0.2f, 0.2f, 0.2f);
        ds_filter.setInputCloud(history_cloud_world);
        ds_filter.filter(*history_ds);

        if (history_ds->empty() || latest_cloud_world->empty()) {
            RCLCPP_WARN(get_logger(), "Loop closure aborted: empty clouds.");
            return;
        }

        // ── 3. Align — both clouds in WORLD frame, initial guess = identity ────
        pcl::PointCloud<pcl::PointXYZ> aligned;
        bool            lc_converged = false;
        double          score        = 0.0;
        Eigen::Matrix4f correction   = Eigen::Matrix4f::Identity();

        if (lc_use_ndt_) {
            ndt_lc_.setInputTarget(history_ds);
            ndt_lc_.setInputSource(latest_cloud_world);
            ndt_lc_.align(aligned, Eigen::Matrix4f::Identity());
            lc_converged = ndt_lc_.hasConverged();
            score        = ndt_lc_.getFitnessScore();
            correction   = ndt_lc_.getFinalTransformation();
        } else {
            vgicp_lc_.setInputSource(latest_cloud_world);
            vgicp_lc_.setInputTarget(history_ds);
            vgicp_lc_.align(aligned, Eigen::Matrix4f::Identity());
            lc_converged = vgicp_lc_.hasConverged();
            score        = vgicp_lc_.getFitnessScore();
            correction   = vgicp_lc_.getFinalTransformation();
        }

        const char* lc_matcher = lc_use_ndt_ ? "NDT" : "VGICP";
        if (!lc_converged) {
            RCLCPP_WARN(get_logger(), "Loop closure %s did not converge.", lc_matcher);
            return;
        }
        Eigen::Vector3f t_corr     = correction.block<3,1>(0,3);
        float correction_dist      = t_corr.norm();
        float correction_angle     = Eigen::AngleAxisf(
            Eigen::Matrix3f(correction.block<3,3>(0,0))).angle() * 180.0f / M_PI;

        RCLCPP_INFO(get_logger(),
            "LC %s: score=%.4f | correction t=%.2fm angle=%.1fdeg",
            lc_matcher, score, correction_dist, correction_angle);

        if (score > lc_fitness_score_) {
            RCLCPP_INFO(get_logger(), "Loop closure refused: score %.4f > threshold %.4f",
                        score, lc_fitness_score_);
            return;
        }

        // Reject if correction is unreasonably large — likely wrong minimum
        // if (correction_dist > 3.0f || correction_angle > 30.0f) {
        //     RCLCPP_WARN(get_logger(),
        //         "Loop closure refused: correction too large (t=%.2fm, angle=%.1fdeg)",
        //         correction_dist, correction_angle);
        //     return;
        // }

        RCLCPP_WARN(get_logger(), "Loop closure accepted! [%s] Score: %.4f | t=%.2fm | angle=%.1fdeg",
                    lc_matcher, score, correction_dist, correction_angle);

        // ── 4. Compute pose constraint — LIO-SAM style ────────────────────────
        // correctionLidarFrame * tWrong = tCorrect
        // poseFrom = tCorrect, poseTo = history pose
        // factor = BetweenFactor(latest, history, poseFrom.between(poseTo))
        Eigen::Matrix4f t_correct = correction * latest_pose_world;

        gtsam::Pose3 pose_from = matrix2Pose3(t_correct);
        gtsam::Pose3 pose_to   = matrix2Pose3(history_pose_world);

        // ── 5. Add factor to graph ────────────────────────────────────────────
        {
            std::lock_guard<std::mutex> gtsam_lock(gtsam_mutex_);
            std::lock_guard<std::mutex> kf_lock   (kf_mutex_);

            // Guard: SLAM restarted while VGICP was running
            if (slam_generation_.load() != generation_before_gicp) {
                RCLCPP_WARN(get_logger(), "Loop closure aborted: SLAM restarted during VGICP.");
                return;
            }

            // Guard: keyframe indices no longer valid
            if (keyframes_.empty() ||
                latest_id  >= static_cast<int>(keyframes_.size()) ||
                closest_id >= static_cast<int>(keyframes_.size())) {
                RCLCPP_WARN(get_logger(), "Loop closure aborted: keyframe count changed.");
                return;
            }

            gtSAMgraph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
                X(latest_id), X(closest_id),
                pose_from.between(pose_to),
                robustLoopNoise_));  // tight + Huber robust kernel

            // LIO-SAM calls update() 5 times for stability on large corrections
            isam_->update(gtSAMgraph_, initialEstimates_);
            isam_->update();
            isam_->update();
            isam_->update();
            isam_->update();
            isam_->update();
            gtSAMgraph_.resize(0);
            initialEstimates_.clear();

            // Set flag — correctPoses() fires in next AddKeyFrame call
            loop_closure_detected_ = true;
            publishLoopConstraints(latest_id, closest_id);
        }
    }

    // ── Submap rebuild — call with kf_mutex_ already held ────────────────────
    void updateSubmap()
    {
        local_map_->clear();
        int start = std::max(0, static_cast<int>(keyframes_.size()) - submap_size_);
        for (int i = start; i < static_cast<int>(keyframes_.size()); ++i) {
            pcl::PointCloud<pcl::PointXYZ> transformed;
            pcl::transformPointCloud(*keyframes_[i].cloud, transformed, keyframes_[i].pose);
            *local_map_ += transformed;
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr ds(new pcl::PointCloud<pcl::PointXYZ>);
        map_filter_.setInputCloud(local_map_);
        map_filter_.filter(*ds);
        local_map_ = ds;
    }

    // ── Full map publisher — call with kf_mutex_ already held ────────────────
    void publishFullMap()
    {
        pcl::PointCloud<pcl::PointXYZ> full_map;
        for (auto & kf : keyframes_) {
            pcl::PointCloud<pcl::PointXYZ> transformed;
            pcl::transformPointCloud(*kf.cloud, transformed, kf.pose);
            full_map += transformed;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setLeafSize(0.2f, 0.2f, 0.2f);
        vg.setInputCloud(full_map.makeShared());
        vg.filter(*downsampled);

        sensor_msgs::msg::PointCloud2 map_msg;
        pcl::toROSMsg(*downsampled, map_msg);
        map_msg.header.frame_id = odom_frame_;
        map_msg.header.stamp    = this->now();
        full_map_pub_->publish(map_msg);

        RCLCPP_DEBUG(get_logger(), "Full map: %zu pts | %zu keyframes",
                     downsampled->size(), keyframes_.size());
    }

    // ── Path publisher — call with kf_mutex_ already held ────────────────────
    void publishPath()
    {
        nav_msgs::msg::Path path_msg;
        path_msg.header.frame_id = odom_frame_;
        path_msg.header.stamp    = this->now();

        for (auto & kf : keyframes_) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header.frame_id = odom_frame_;
            ps.header.stamp    = path_msg.header.stamp;

            Eigen::Vector3f    t(kf.pose.block<3,1>(0,3));
            Eigen::Quaternionf q(kf.pose.block<3,3>(0,0));

            ps.pose.position.x    = t.x();
            ps.pose.position.y    = t.y();
            ps.pose.position.z    = t.z();
            ps.pose.orientation.x = q.x();
            ps.pose.orientation.y = q.y();
            ps.pose.orientation.z = q.z();
            ps.pose.orientation.w = q.w();

            path_msg.poses.push_back(ps);
        }

        path_pub_->publish(path_msg);
    }

    // ── EKF callback ──────────────────────────────────────────────────────────
    void ekfCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
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

        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        pose.block<3,3>(0,0) = q.toRotationMatrix();
        pose.block<3,1>(0,3) = t;

        std::lock_guard<std::mutex> lock(ekf_mutex_);
        latest_ekf_pose_ = pose;
        latest_ekf_stamp_ = msg->header.stamp;
        has_ekf_ = true;
        ekf_msg_count_++;
    }

    // ── Point cloud callback ───────────────────────────────────────────────────
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (!has_ekf_) {
            RCLCPP_WARN_ONCE(get_logger(), "Waiting for first EKF message...");
            return;
        }
        {
            std::lock_guard<std::mutex> lock(ekf_mutex_);
            if (ekf_msg_count_ < min_ekf_msgs_) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                    "Waiting for EKF to converge (%d / %d messages)",
                    ekf_msg_count_.load(), min_ekf_msgs_);
                return;
            }
        }

        {
            std::lock_guard<std::mutex> lock(ekf_mutex_);
            double dt = std::abs((rclcpp::Time(msg->header.stamp) - rclcpp::Time(latest_ekf_stamp_)).seconds());
            if (dt > ekf_max_age_) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "EKF message is %.3fs old (threshold %.3fs) — skipping scan", dt, ekf_max_age_);
                return;
            }
        }

        // 1. Convert and intensity-filter
        pcl::PointCloud<pcl::PointXYZ>::Ptr raw(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *raw);

        // pcl::PointCloud<pcl::PointXYZ>::Ptr intensity_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        // intensity_filtered->reserve(raw->size());
        // if (filter_intensity) {
        //     for (const auto & pt : *raw)
        //         if (std::isfinite(pt.x) && pt.intensity > min_intensity)
        //             intensity_filtered->push_back(pt);
        // } else {
        //     *intensity_filtered = *raw;
        // }
        // 2. Transform into base frame
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_base(new pcl::PointCloud<pcl::PointXYZ>);
        Eigen::Matrix4f base_transform = is_ned_ ? (ned_transform_ * base2sonar_) : base2sonar_;
        // pcl::transformPointCloud(*intensity_filtered, *cloud_base, base_transform);
        pcl::transformPointCloud(*raw, *cloud_base, base_transform);


        // 3. Radius outlier removal, then statistical outlier removal
        pcl::PointCloud<pcl::PointXYZ>::Ptr ror_out(new pcl::PointCloud<pcl::PointXYZ>);
        if (filter_radius_outliers_) {
            ror_.setInputCloud(cloud_base);
            ror_.filter(*ror_out);
        } else {
            ror_out = cloud_base;
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
        if (filter_outliers) {
            sor_.setInputCloud(ror_out);
            sor_.filter(*filtered);
        } else {
            filtered = ror_out;
        }

        // 4. Grab EKF snapshot
        Eigen::Matrix4f current_ekf_pose;
        {
            std::lock_guard<std::mutex> lock(ekf_mutex_);
            current_ekf_pose = latest_ekf_pose_;
        }

        // 5. Bootstrap
        bool map_empty;
        {
            std::lock_guard<std::mutex> kf_lock(kf_mutex_);
            map_empty = local_map_->empty();
        }

        if (map_empty) {
            {
                std::lock_guard<std::mutex> pose_lock(pose_mutex_);
                global_pose_ = current_ekf_pose;
            }
            prev_ekf_pose_ = current_ekf_pose;
            AddKeyFrame(current_ekf_pose, filtered, current_ekf_pose(2, 3));
            return;
        }

        // 6. Build initial guess from EKF dead-reckoning
        Eigen::Matrix4f current_global;
        {
            std::lock_guard<std::mutex> pose_lock(pose_mutex_);
            current_global = global_pose_;
        }
        Eigen::Matrix4f ekf_delta     = prev_ekf_pose_.inverse() * current_ekf_pose;
        Eigen::Matrix4f initial_guess = current_global * ekf_delta;

        // 7. Snapshot local map
        pcl::PointCloud<pcl::PointXYZ>::Ptr map_snapshot;
        {
            std::lock_guard<std::mutex> kf_lock(kf_mutex_);
            map_snapshot = local_map_;
        }

        // 8. Run GICP
        vgicp_.setInputTarget(map_snapshot);
        vgicp_.setInputSource(filtered);

        pcl::PointCloud<pcl::PointXYZ> aligned;
        vgicp_.align(aligned, initial_guess);

        if (vgicp_.hasConverged()) {
            Eigen::Matrix4f result = vgicp_.getFinalTransformation();

            Eigen::Matrix4f diff           = initial_guess.inverse() * result;
            float correction_dist          = diff.block<3,1>(0,3).norm();
            float correction_angle         = Eigen::AngleAxisf(
                Eigen::Matrix3f(diff.block<3,3>(0,0))).angle() * 180.0f / M_PI;

            RCLCPP_DEBUG(get_logger(),
                "GICP | score: %.4f | correction: t=%.2fm angle=%.1fdeg | src: %zu | tgt: %zu",
                vgicp_.getFitnessScore(), correction_dist, correction_angle,
                filtered->size(), map_snapshot->size());

            lost_frames_ = 0;

            {
                std::lock_guard<std::mutex> pose_lock(pose_mutex_);
                if (ekf_z_) result(2, 3) = current_ekf_pose(2, 3);
                global_pose_   = result;
                current_global = result;
            }
            double score = vgicp_.getFitnessScore();
            publishOdometry(msg->header, score, false);
            AddKeyFrame(current_global, filtered, current_ekf_pose(2, 3));

            if (map_pub_count_++ % 5 == 0) {
                sensor_msgs::msg::PointCloud2 map_msg;
                {
                    std::lock_guard<std::mutex> kf_lock(kf_mutex_);
                    pcl::toROSMsg(*local_map_, map_msg);
                }
                map_msg.header.frame_id = odom_frame_;
                map_msg.header.stamp    = msg->header.stamp;
                global_map_pub_->publish(map_msg);
            }

            prev_ekf_pose_ = current_ekf_pose;

        } else {
            if (lost_frames_ < max_lost_frames) {
                ++lost_frames_;
                std::lock_guard<std::mutex> pose_lock(pose_mutex_);
                global_pose_   = initial_guess;
                prev_ekf_pose_ = current_ekf_pose;
            } else {
                RCLCPP_WARN(get_logger(), "Tracking lost — restarting SLAM.");
                lost_frames_ = 0;

                {
                    std::lock_guard<std::mutex> pose_lock(pose_mutex_);
                    global_pose_ = current_ekf_pose;
                }
                prev_ekf_pose_ = current_ekf_pose;

                {
                    std::lock_guard<std::mutex> gtsam_lock(gtsam_mutex_);
                    std::lock_guard<std::mutex> kf_lock   (kf_mutex_);
                    slam_generation_++;
                    initGTSAM();
                    keyframes_.clear();
                    local_map_->clear();
                }

                map_pub_count_ = 0;
            }
        }
    }

    // ── Odometry publisher ────────────────────────────────────────────────────
    void publishOdometry(const std_msgs::msg::Header & header, double fitness_score = 0.1, bool is_global_match = false)
    {
        Eigen::Matrix4f pose;
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            pose = global_pose_;
        }

        Eigen::Vector3f    t(pose.block<3,1>(0,3));
        Eigen::Quaternionf q(pose.block<3,3>(0,0));

        nav_msgs::msg::Odometry odom;
        odom.header.stamp    = header.stamp;
        odom.header.frame_id = odom_frame_;
        odom.child_frame_id  = base_frame_;
        odom.pose.pose.position.x    = t.x();
        odom.pose.pose.position.y    = t.y();
        odom.pose.pose.position.z    = t.z();
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();
        

        double base  = is_global_match ? 0.01 : 0.5;
        double cov_pos = base + fitness_score * 10.0;
        double cov_yaw = cov_pos * 0.5;

        odom.pose.covariance.fill(0.0);
        odom.pose.covariance[0]  = cov_pos;    // x
        odom.pose.covariance[7]  = cov_pos;    // y
        odom.pose.covariance[14] = cov_pos;    // z
        odom.pose.covariance[21] = 9999.0;     
        odom.pose.covariance[28] = 9999.0;     
        odom.pose.covariance[35] = cov_yaw;    // yaw


        odom_pub_->publish(odom);
    }

    void publishLoopConstraints(int latest_id, int closest_id)
    {
        visualization_msgs::msg::MarkerArray marker_array;

        // Line connecting the two keyframes
        visualization_msgs::msg::Marker line;
        line.header.frame_id = odom_frame_;
        line.header.stamp    = this->now();
        line.ns              = "loop_edges";
        line.id              = latest_id;   // unique per loop closure
        line.type            = visualization_msgs::msg::Marker::LINE_STRIP;
        line.action          = visualization_msgs::msg::Marker::ADD;
        line.scale.x         = 0.1;         // line width in meters
        line.color.r         = 0.0f;
        line.color.g         = 1.0f;        // green — same as LIO-SAM
        line.color.b         = 0.0f;
        line.color.a         = 1.0f;
        line.pose.orientation.w = 1.0;

        geometry_msgs::msg::Point p1, p2;
        p1.x = keyframes_[latest_id].pose(0,3);
        p1.y = keyframes_[latest_id].pose(1,3);
        p1.z = keyframes_[latest_id].pose(2,3);

        p2.x = keyframes_[closest_id].pose(0,3);
        p2.y = keyframes_[closest_id].pose(1,3);
        p2.z = keyframes_[closest_id].pose(2,3);

        line.points.push_back(p1);
        line.points.push_back(p2);
        marker_array.markers.push_back(line);

        // Sphere at the history keyframe (where the loop closes to)
        visualization_msgs::msg::Marker sphere;
        sphere.header.frame_id = odom_frame_;
        sphere.header.stamp    = this->now();
        sphere.ns              = "loop_nodes";
        sphere.id              = latest_id;
        sphere.type            = visualization_msgs::msg::Marker::SPHERE;
        sphere.action          = visualization_msgs::msg::Marker::ADD;
        sphere.scale.x         = 0.3;
        sphere.scale.y         = 0.3;
        sphere.scale.z         = 0.3;
        sphere.color.r         = 1.0f;
        sphere.color.g         = 0.0f;
        sphere.color.b         = 0.0f;  // red sphere at loop node
        sphere.color.a         = 1.0f;
        sphere.pose.orientation.w = 1.0;
        sphere.pose.position.x = p2.x;
        sphere.pose.position.y = p2.y;
        sphere.pose.position.z = p2.z;
        marker_array.markers.push_back(sphere);

        lc_marker_pub_->publish(marker_array);
    }

    // ── ROS interfaces ────────────────────────────────────────────────────────
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr        ekf_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr           odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr     global_map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr     full_map_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr               path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr lc_marker_pub_;
    std::thread                                                     loop_closure_thread_;

    // ── GICP / NDT ────────────────────────────────────────────────────────────
    fast_gicp::FastVGICP<pcl::PointXYZ, pcl::PointXYZ>        vgicp_;
    fast_gicp::FastVGICP<pcl::PointXYZ, pcl::PointXYZ>        vgicp_lc_;
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_lc_;

    // ── Map ───────────────────────────────────────────────────────────────────
    pcl::PointCloud<pcl::PointXYZ>::Ptr          local_map_;
    pcl::VoxelGrid<pcl::PointXYZ>                map_filter_;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor_;
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror_;

    // ── GTSAM — always lock gtsam_mutex_ BEFORE kf_mutex_ ────────────────────
    std::mutex                              gtsam_mutex_;
    std::unique_ptr<gtsam::ISAM2>           isam_;
    gtsam::NonlinearFactorGraph             gtSAMgraph_;
    gtsam::Values                           initialEstimates_;
    gtsam::noiseModel::Diagonal::shared_ptr priorNoise_;
    gtsam::noiseModel::Diagonal::shared_ptr odomNoise_;
    gtsam::noiseModel::Robust::shared_ptr   robustLoopNoise_;  // FIX: was missing

    // ── Loop closure flag — same as LIO-SAM's aLoopIsClosed ──────────────────
    std::atomic<bool> loop_closure_detected_{false};

    // ── State ─────────────────────────────────────────────────────────────────
    std::mutex      pose_mutex_;
    Eigen::Matrix4f global_pose_;

    std::mutex           ekf_mutex_;
    Eigen::Matrix4f      latest_ekf_pose_;
    rclcpp::Time         latest_ekf_stamp_{0, 0, RCL_ROS_TIME};
    bool                 has_ekf_{false};
    std::atomic<int>     ekf_msg_count_{0};
    int                  min_ekf_msgs_{20};
    double               ekf_max_age_{0.5};

    Eigen::Matrix4f prev_ekf_pose_;  // only accessed from pointCloudCallback

    std::mutex            kf_mutex_;
    std::vector<Keyframe> keyframes_;

    std::atomic<int> slam_generation_{0};

    // ── Extrinsics ────────────────────────────────────────────────────────────
    Eigen::Matrix4f sonar2base_, base2sonar_, ned_transform_;

    // ── Config ────────────────────────────────────────────────────────────────
    std::string odom_frame_, base_frame_;
    bool        is_ned_{false};

    int    submap_size_{20};
    double kf_dist_thresh_{0.5}, kf_angle_thresh_{10.0};

    double lc_search_radius_{10.0}, lc_fitness_score_{0.3};
    int    lc_history_gap_{10}, lc_submap_size_{7};
    bool   lc_use_ndt_{false};

    // GICP sanity check thresholds
    double gicp_max_correction_dist_{1.0};   // meters
    double gicp_max_correction_angle_{15.0}; // degrees

    int lost_frames_{0};
    int max_lost_frames{50};
    int map_pub_count_{0};

    bool filter_outliers;
    bool filter_radius_outliers_;
    bool filter_intensity;
    double min_intensity;
    bool ekf_z_{false};
    bool loop_ekf_z_{false};

    double odom_noise_roll_{0.1},  odom_noise_pitch_{0.1}, odom_noise_yaw_{0.3};
    double odom_noise_x_{0.5},     odom_noise_y_{0.5},     odom_noise_z_{0.3};

    double lc_noise_roll_{0.01},   lc_noise_pitch_{0.01},  lc_noise_yaw_{0.01};
    double lc_noise_x_{0.05},      lc_noise_y_{0.05},      lc_noise_z_{0.05};
    double lc_huber_k_{1.0};



};

// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;

    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<GicpOdomNode>(options);
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}