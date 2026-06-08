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
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
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
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/PreintegrationParams.h>
#include <gtsam/navigation/AttitudeFactor.h>
#include <gtsam/base/numericalDerivative.h>

#include <sensor_msgs/msg/imu.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

using gtsam::symbol_shorthand::X;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::B;

// ── DVL Velocity Factor ───────────────────────────────────────────────────────
// Ports equation (6) / rv from AQUA-SLAM paper.
// Constrains V(i) using DVL velocity measured in body (DVL) frame.
// Residual: v_measured_body - R^T * v_world
//
// Variables: Pose3 (for rotation), Vector3 (world-frame velocity)
class DvlVelocityFactor : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Vector3>
{
    using Base = gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Vector3>;
    gtsam::Vector3 v_measured_body_;  // DVL measurement in body/DVL frame
public:
    DvlVelocityFactor(gtsam::Key pose_key, gtsam::Key vel_key,
                      const gtsam::Vector3 & v_body,
                      const gtsam::SharedNoiseModel & model)
        : Base(model, pose_key, vel_key), v_measured_body_(v_body) {}

    gtsam::Vector evaluateError(
        const gtsam::Pose3 & pose,
        const gtsam::Vector3 & vel_world,
        boost::optional<gtsam::Matrix&> H1 = boost::none,
        boost::optional<gtsam::Matrix&> H2 = boost::none) const override
    {
        auto f = [this](const gtsam::Pose3& p, const gtsam::Vector3& v) -> gtsam::Vector3 {
            return p.rotation().matrix().transpose() * v - v_measured_body_;
        };
        if (H1) *H1 = gtsam::numericalDerivative21<gtsam::Vector3, gtsam::Pose3, gtsam::Vector3>(f, pose, vel_world);
        if (H2) *H2 = gtsam::numericalDerivative22<gtsam::Vector3, gtsam::Pose3, gtsam::Vector3>(f, pose, vel_world);
        return f(pose, vel_world);
    }
};

// ── DVL Translation Factor ────────────────────────────────────────────────────
// Ports equation (8) / rt from AQUA-SLAM paper.
// Constrains (Pose_i, Pose_j) using DVL pre-integrated translation.
//
// The DVL pre-integration ΔDi_p̄_DiDj is computed ONCE between keyframes by
// accumulating: Σ ΔR̂_IiIk * R_ID * Di_v * Δt  (gyro rotates each DVL sample)
// and stored here. The optimizer only evaluates hDt(Xi, Xj) each iteration.
//
// Residual (3D): ΔDi_p̄_DiDj - hDt(Xi, Xj)
// where hDt = R_ID * [D_pDC - R_DC * Ri^T * Rj * R_DC^T * D_pDC
//                            + R_DC * (Ri^T * pj - Ri^T * pi)]
//
// Variables: Pose3 at keyframe i, Pose3 at keyframe j
class DvlTranslationFactor : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>
{
    using Base = gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>;

    Eigen::Vector3d preint_dp_;   // ΔDi_p̄_DiDj — pre-integrated DVL translation
    Eigen::Matrix3d R_ID_;        // rotation: IMU → DVL frame
    Eigen::Matrix3d R_DC_;        // rotation: DVL → camera frame
    Eigen::Vector3d D_pDC_;       // DVL-to-camera translation expressed in DVL frame

public:
    DvlTranslationFactor(gtsam::Key pose_i_key, gtsam::Key pose_j_key,
                         const Eigen::Vector3d & preint_dp,
                         const Eigen::Matrix3d & R_ID,
                         const Eigen::Matrix3d & R_DC,
                         const Eigen::Vector3d & D_pDC,
                         const gtsam::SharedNoiseModel & model)
        : Base(model, pose_i_key, pose_j_key)
        , preint_dp_(preint_dp), R_ID_(R_ID), R_DC_(R_DC), D_pDC_(D_pDC) {}

    gtsam::Vector evaluateError(
        const gtsam::Pose3 & pose_i,
        const gtsam::Pose3 & pose_j,
        boost::optional<gtsam::Matrix&> H1 = boost::none,
        boost::optional<gtsam::Matrix&> H2 = boost::none) const override
    {
        auto f = [this](const gtsam::Pose3& pi, const gtsam::Pose3& pj) -> gtsam::Vector3 {
            Eigen::Matrix3d Ri  = pi.rotation().matrix();
            Eigen::Matrix3d Rj  = pj.rotation().matrix();
            Eigen::Matrix3d RiT = Ri.transpose();
            Eigen::Vector3d h_Dt =
                R_ID_ * (D_pDC_
                       - R_DC_ * RiT * Rj * R_DC_.transpose() * D_pDC_
                       + R_DC_ * (RiT * pj.translation() - RiT * pi.translation()));
            return preint_dp_ - h_Dt;
        };
        if (H1) *H1 = gtsam::numericalDerivative21<gtsam::Vector3, gtsam::Pose3, gtsam::Pose3>(f, pose_i, pose_j);
        if (H2) *H2 = gtsam::numericalDerivative22<gtsam::Vector3, gtsam::Pose3, gtsam::Pose3>(f, pose_i, pose_j);
        return f(pose_i, pose_j);
    }
};

// ── Depth Factor ─────────────────────────────────────────────────────────────
// Constrains the Z translation of Pose3 to a depth measurement (scalar).
// Residual: pose.translation().z() - z_measured
// Jacobian: [0 0 0 | 0 0 1]  (only the z-translation DoF)
class DepthFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3>
{
    using Base = gtsam::NoiseModelFactor1<gtsam::Pose3>;
    double z_measured_;
public:
    DepthFactor(gtsam::Key pose_key, double z_measured,
                const gtsam::SharedNoiseModel & model)
        : Base(model, pose_key), z_measured_(z_measured) {}

    gtsam::Vector evaluateError(
        const gtsam::Pose3 & pose,
        boost::optional<gtsam::Matrix&> H = boost::none) const override
    {
        auto f = [this](const gtsam::Pose3& p) -> gtsam::Vector1 {
            return (gtsam::Vector1() << p.translation().z() - z_measured_).finished();
        };
        if (H) *H = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Pose3>(f, pose);
        return f(pose);
    }
};

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

        map_res_              = this->declare_parameter<double>("tuning.map_res",          0.1);
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
        use_lc_ = this->declare_parameter<bool>("loop_closure.use_lc", true);

        lc_use_ndt_      = this->declare_parameter<bool>  ("loop_closure.use_ndt",         false);
        double lc_ndt_res  = this->declare_parameter<double>("loop_closure.ndt_resolution",  2.0);
        double lc_ndt_step = this->declare_parameter<double>("loop_closure.ndt_step_size",   0.5);
        double lc_ndt_eps  = this->declare_parameter<double>("loop_closure.ndt_epsilon",     0.01);
        int    lc_ndt_iter = this->declare_parameter<int>  ("loop_closure.ndt_max_iter",     50);

        odom_frame_    = this->declare_parameter<std::string>("frames.odom_frame", "odom");
        base_frame_    = this->declare_parameter<std::string>("frames.base_frame", "sam_auv_v1/base_link");
        min_intensity = static_cast<float>(this->declare_parameter<double>("tuning.min_intensity", 0.0));
        is_ned_        = this->declare_parameter<bool>("is_ned", false);
        use_ekf_       = this->declare_parameter<bool>("use_ekf", true);
        ekf_z_         = this->declare_parameter<bool>("ekf_z", false);
        loop_ekf_z_    = this->declare_parameter<bool>("loop_ekf_z", false);
        ekf_max_age_   = this->declare_parameter<double>("ekf_max_age", 0.1);

        debug        = this->declare_parameter<bool>("debug",       false);
        publish_tf_  = this->declare_parameter<bool>("publish_tf",  false);
        if (publish_tf_) {
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
            RCLCPP_INFO(get_logger(), "TF broadcasting enabled (%s -> %s)", odom_frame_.c_str(), base_frame_.c_str());
        }

        submap_size_     = this->declare_parameter<int>   ("keyframe.submap_size",  20);
        kf_dist_thresh_  = this->declare_parameter<double>("keyframe.dist_thresh",  0.5);
        kf_angle_thresh_ = this->declare_parameter<double>("keyframe.angle_thresh", 10.0);
        kf_min_dt_       = this->declare_parameter<double>("keyframe.min_dt", 0.3);

        lc_search_radius_        = this->declare_parameter<double>("loop_closure.search_radius",        10.0);
        lc_fitness_score_        = this->declare_parameter<double>("loop_closure.fitness_score",        0.3);
        lc_max_correction_dist_  = this->declare_parameter<double>("loop_closure.max_correction_dist",  3.0);
        lc_max_correction_angle_ = this->declare_parameter<double>("loop_closure.max_correction_angle", 30.0);
        lc_history_gap_   = this->declare_parameter<int>   ("loop_closure.history_gap",   10);
        lc_submap_size_   = this->declare_parameter<int>   ("loop_closure.submap_size",   7);

        // GICP sanity check thresholds
        gicp_max_correction_dist_  = this->declare_parameter<double>("tuning.gicp_max_correction_dist",  1.0);
        gicp_max_correction_angle_ = this->declare_parameter<double>("tuning.gicp_max_correction_angle", 15.0);
        gicp_fitness_score_        = this->declare_parameter<double>("tuning.gicp_fitness_score",         0.0);  // 0 = disabled

        // Extrinsics: translation + RPY in degrees (sonar -> base)
        double b2s_t_x   = this->declare_parameter<double>("tf.base2sonar_x",      -0.545);
        double b2s_t_y   = this->declare_parameter<double>("tf.base2sonar_y",       0.000);
        double b2s_t_z   = this->declare_parameter<double>("tf.base2sonar_z",      -0.404);
        double b2s_roll  = this->declare_parameter<double>("tf.base2sonar_roll",    0.0);
        double b2s_pitch = this->declare_parameter<double>("tf.base2sonar_pitch",  -30.0);
        double b2s_yaw   = this->declare_parameter<double>("tf.base2sonar_yaw",     0.0);

        Eigen::Quaternionf rotation_sb;
        rotation_sb = Eigen::AngleAxisf(static_cast<float>(b2s_yaw   * M_PI / 180.0), Eigen::Vector3f::UnitZ())
                    * Eigen::AngleAxisf(static_cast<float>(b2s_pitch  * M_PI / 180.0), Eigen::Vector3f::UnitY())
                    * Eigen::AngleAxisf(static_cast<float>(b2s_roll   * M_PI / 180.0), Eigen::Vector3f::UnitX());

        base2sonar_ = Eigen::Matrix4f::Identity();
        base2sonar_.block<3,3>(0,0) = rotation_sb.toRotationMatrix();
        base2sonar_.block<3,1>(0,3) = Eigen::Vector3f(
            static_cast<float>(b2s_t_x),
            static_cast<float>(b2s_t_y),
            static_cast<float>(b2s_t_z));
        sonar2base_ = base2sonar_.inverse();

        ned_transform_ << 0, -1,  0, 0,
                          1,  0,  0, 0,
                          0,  0, -1, 0,
                          0,  0,  0, 1;

        RCLCPP_INFO(get_logger(),
            "Sonar->Base extrinsics: t=[%.3f, %.3f, %.3f] rpy=[%.1f, %.1f, %.1f] deg",
            b2s_t_x, b2s_t_y, b2s_t_z, b2s_roll, b2s_pitch, b2s_yaw);

        // IMU extrinsics — rotation only (translation is negligible for slow AUVs)
        double i2b_roll  = this->declare_parameter<double>("tf.imu2base_roll",  0.0);
        double i2b_pitch = this->declare_parameter<double>("tf.imu2base_pitch", 0.0);
        double i2b_yaw   = this->declare_parameter<double>("tf.imu2base_yaw",   0.0);

        Eigen::Quaterniond rotation_ib;
        rotation_ib = Eigen::AngleAxisd(i2b_yaw   * M_PI / 180.0, Eigen::Vector3d::UnitZ())
                    * Eigen::AngleAxisd(i2b_pitch * M_PI / 180.0, Eigen::Vector3d::UnitY())
                    * Eigen::AngleAxisd(i2b_roll  * M_PI / 180.0, Eigen::Vector3d::UnitX());
        R_imu2base_ = rotation_ib.toRotationMatrix();

        RCLCPP_INFO(get_logger(),
            "IMU->Base extrinsics: rpy=[%.1f, %.1f, %.1f] deg",
            i2b_roll, i2b_pitch, i2b_yaw);

        // DVL lever arm — translation from base_link to dvl_link in body frame
        // Used to remove the ω×r velocity component from DVL readings
        r_base2dvl_.x() = this->declare_parameter<double>("tf.dvl2base_x", 0.0);
        r_base2dvl_.y() = this->declare_parameter<double>("tf.dvl2base_y", 0.0);
        r_base2dvl_.z() = this->declare_parameter<double>("tf.dvl2base_z", 0.0);
        RCLCPP_INFO(get_logger(),
            "DVL lever arm (base->dvl): [%.3f, %.3f, %.3f] m",
            r_base2dvl_.x(), r_base2dvl_.y(), r_base2dvl_.z());

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
        map_filter_.setLeafSize(map_res_, map_res_, map_res_);

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

        // Visual odometry factor
        use_vo_           = this->declare_parameter<bool>  ("vo.use_vo",       false);
        vo_max_delta_     = this->declare_parameter<double>("vo.max_delta",     2.0);   // max translation between KFs [m]
        vo_reset_thresh_  = this->declare_parameter<double>("vo.reset_thresh",  0.05);  // position norm below this → reset
        {
            double nx = this->declare_parameter<double>("vo.noise_x",     0.05);
            double ny = this->declare_parameter<double>("vo.noise_y",     0.05);
            double nz = this->declare_parameter<double>("vo.noise_z",     0.1);
            double nr = this->declare_parameter<double>("vo.noise_roll",  0.01);
            double np = this->declare_parameter<double>("vo.noise_pitch", 0.01);
            double nyw= this->declare_parameter<double>("vo.noise_yaw",   0.05);
            voNoise_ = gtsam::noiseModel::Diagonal::Sigmas(
                (gtsam::Vector(6) << nr, np, nyw, nx, ny, nz).finished());
        }
        if (use_vo_) {
            std::string vo_topic = this->declare_parameter<std::string>("topics.vo_sub", "/odometry/visual");
            vo_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                vo_topic, 10,
                std::bind(&GicpOdomNode::voCallback, this, std::placeholders::_1));
            RCLCPP_INFO(get_logger(), "Visual odometry factor enabled on %s", vo_topic.c_str());
        }

        // IMU parameters
        std::string imu_topic = this->declare_parameter<std::string>("topics.imu_sub", "/imu/data");
        imu_accel_noise_  = this->declare_parameter<double>("imu.accel_noise",       0.05);
        imu_gyro_noise_   = this->declare_parameter<double>("imu.gyro_noise",        0.005);
        imu_accel_bias_   = this->declare_parameter<double>("imu.accel_bias_noise",  1e-3);
        imu_gyro_bias_    = this->declare_parameter<double>("imu.gyro_bias_noise",   1e-3);
        imu_gravity_      = this->declare_parameter<double>("imu.gravity",           9.81);
        use_imu_          = this->declare_parameter<bool>  ("imu.use_imu",           false);
        imu_start_kf_     = this->declare_parameter<int>   ("imu.start_kf",          0);
        // Static bias offsets — measure by echoing IMU at rest; subtracted per-sample
        imu_accel_bias_x_ = this->declare_parameter<double>("imu.static_accel_bias_x", 0.0);
        imu_accel_bias_y_ = this->declare_parameter<double>("imu.static_accel_bias_y", 0.0);
        imu_accel_bias_z_ = this->declare_parameter<double>("imu.static_accel_bias_z", 0.0);
        imu_gyro_bias_x_  = this->declare_parameter<double>("imu.static_gyro_bias_x",  0.0);
        imu_gyro_bias_y_  = this->declare_parameter<double>("imu.static_gyro_bias_y",  0.0);
        imu_gyro_bias_z_  = this->declare_parameter<double>("imu.static_gyro_bias_z",  0.0);

        // DVL parameters
        std::string dvl_topic = this->declare_parameter<std::string>("topics.dvl_sub", "/dvl/odometry");
        dvl_noise_x_  = this->declare_parameter<double>("dvl.noise_x",  0.05);
        dvl_noise_y_  = this->declare_parameter<double>("dvl.noise_y",  0.05);
        dvl_noise_z_  = this->declare_parameter<double>("dvl.noise_z",  0.05);
        use_dvl_       = this->declare_parameter<bool>("dvl.use_dvl",       false);
        use_dvl_trans_ = this->declare_parameter<bool>("dvl.use_dvl_trans", false);
        dvl_max_vel_   = this->declare_parameter<double>("dvl.max_vel", 3.0);  // reject readings above this [m/s]
        imu_max_accel_ = this->declare_parameter<double>("imu.max_accel", 50.0);  // reject spikes above this [m/s^2]
        imu_max_gyro_  = this->declare_parameter<double>("imu.max_gyro",  10.0);  // reject spikes above this [rad/s]

        // DVL translation factor noise (pre-integrated position constraint between keyframes)
        double dvl_trans_nx = this->declare_parameter<double>("dvl.trans_noise_x", 0.1);
        double dvl_trans_ny = this->declare_parameter<double>("dvl.trans_noise_y", 0.1);
        double dvl_trans_nz = this->declare_parameter<double>("dvl.trans_noise_z", 0.1);
        dvlTransNoise_ = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(3) << dvl_trans_nx, dvl_trans_ny, dvl_trans_nz).finished());

        // DVL–IMU–camera extrinsic calibration (RPY in degrees, translation in metres)
        // R_ID: rotation from IMU frame → DVL frame
        {
            double roll  = this->declare_parameter<double>("tf.imu2dvl_roll",  0.0);
            double pitch = this->declare_parameter<double>("tf.imu2dvl_pitch", 0.0);
            double yaw   = this->declare_parameter<double>("tf.imu2dvl_yaw",   0.0);
            R_ID_ = (Eigen::AngleAxisd(yaw   * M_PI/180.0, Eigen::Vector3d::UnitZ())
                   * Eigen::AngleAxisd(pitch * M_PI/180.0, Eigen::Vector3d::UnitY())
                   * Eigen::AngleAxisd(roll  * M_PI/180.0, Eigen::Vector3d::UnitX()))
                    .toRotationMatrix();
        }
        // R_DC: rotation from DVL frame → camera frame
        {
            double roll  = this->declare_parameter<double>("tf.dvl2cam_roll",  0.0);
            double pitch = this->declare_parameter<double>("tf.dvl2cam_pitch", 0.0);
            double yaw   = this->declare_parameter<double>("tf.dvl2cam_yaw",   0.0);
            R_DC_ = (Eigen::AngleAxisd(yaw   * M_PI/180.0, Eigen::Vector3d::UnitZ())
                   * Eigen::AngleAxisd(pitch * M_PI/180.0, Eigen::Vector3d::UnitY())
                   * Eigen::AngleAxisd(roll  * M_PI/180.0, Eigen::Vector3d::UnitX()))
                    .toRotationMatrix();
        }
        // D_pDC: translation DVL → camera expressed in DVL frame
        D_pDC_.x() = this->declare_parameter<double>("tf.dvl2cam_x", 0.0);
        D_pDC_.y() = this->declare_parameter<double>("tf.dvl2cam_y", 0.0);
        D_pDC_.z() = this->declare_parameter<double>("tf.dvl2cam_z", 0.0);
        RCLCPP_INFO(get_logger(), "DVL extrinsics loaded. D_pDC=[%.3f,%.3f,%.3f]",
            D_pDC_.x(), D_pDC_.y(), D_pDC_.z());

        // AHRS attitude parameters
        use_ahrs_       = this->declare_parameter<bool>  ("ahrs.use_ahrs",    false);
        ahrs_noise_rp_  = this->declare_parameter<double>("ahrs.noise_rp",    0.02); // ~1°
        ahrs_noise_yaw_ = this->declare_parameter<double>("ahrs.noise_yaw",   0.1);  // ~6°

        // Accelerometer gravity prior — constrains roll+pitch from low-pass-filtered accel
        // Prevents gravity from leaking onto horizontal axes when AHRS is off
        use_accel_gravity_   = this->declare_parameter<bool>  ("ahrs.use_accel_gravity",  false);
        accel_gravity_noise_ = this->declare_parameter<double>("ahrs.accel_gravity_noise", 0.1);  // [rad]
        if (use_accel_gravity_) {
            RCLCPP_INFO(get_logger(),
                "Accelerometer gravity prior enabled (ORB-SLAM3 style): noise=%.4f rad",
                accel_gravity_noise_);
        }

        // Depth factor parameters
        use_depth_      = this->declare_parameter<bool>  ("depth.use_depth",  false);
        depth_noise_    = this->declare_parameter<double>("depth.noise",       0.05); // [m]

        initGTSAM();

        // IMU preintegration setup
        if (use_imu_) {
            // MakeSharedD = Z-down world frame: gravity = (0,0,+g) in world.
            // Correct for this sensor: a_z ≈ -9.83 at rest → Z axis points DOWN.
            auto imu_p = gtsam::PreintegrationParams::MakeSharedD(imu_gravity_);
            imu_p->accelerometerCovariance = gtsam::I_3x3 * imu_accel_noise_ * imu_accel_noise_;
            imu_p->gyroscopeCovariance     = gtsam::I_3x3 * imu_gyro_noise_  * imu_gyro_noise_;
            imu_p->integrationCovariance   = gtsam::I_3x3 * 1e-6;
            preint_      = std::make_shared<gtsam::PreintegratedImuMeasurements>(imu_p, gtsam::imuBias::ConstantBias());
            scan_preint_ = std::make_shared<gtsam::PreintegratedImuMeasurements>(imu_p, gtsam::imuBias::ConstantBias());

            auto imu_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            rclcpp::SubscriptionOptions imu_sub_opt;
            imu_sub_opt.callback_group = imu_cb_group;
            imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
                imu_topic, 200,
                std::bind(&GicpOdomNode::imuCallback, this, std::placeholders::_1),
                imu_sub_opt);
            RCLCPP_INFO(get_logger(), "IMU preintegration enabled on %s", imu_topic.c_str());
        }

        if (use_dvl_) {
            auto dvl_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            rclcpp::SubscriptionOptions dvl_sub_opt;
            dvl_sub_opt.callback_group = dvl_cb_group;
            dvl_sub_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
                dvl_topic, 50,
                std::bind(&GicpOdomNode::dvlCallback, this, std::placeholders::_1),
                dvl_sub_opt);
            RCLCPP_INFO(get_logger(), "DVL velocity factor enabled on %s", dvl_topic.c_str());
        }

        std::string ahrs_topic = this->declare_parameter<std::string>("topics.ahrs_sub", "/dvl/ahrs_imu");
        ahrs_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            ahrs_topic, 50,
            [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
                gtsam::Rot3 R = gtsam::Rot3::Quaternion(
                    msg->orientation.w, msg->orientation.x,
                    msg->orientation.y, msg->orientation.z);
                std::lock_guard<std::mutex> lk(ahrs_mutex_);
                latest_ahrs_rot_ = R;
                has_ahrs_ = true;
            });
        if (use_ahrs_) {
            RCLCPP_INFO(get_logger(), "AHRS attitude factor enabled on %s", ahrs_topic.c_str());
        }

        if (use_depth_) {
            std::string depth_topic = this->declare_parameter<std::string>(
                "topics.depth_sub", "/depth_odom");
            depth_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                depth_topic, 50,
                [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
                    std::lock_guard<std::mutex> lk(depth_mutex_);
                    latest_depth_z_ = msg->pose.pose.position.z;
                    has_depth_ = true;
                });
            RCLCPP_INFO(get_logger(), "Depth factor enabled on %s", depth_topic.c_str());
        }

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

        // IMU bias random walk noise
        biasBetweenNoise_ = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << imu_accel_bias_, imu_accel_bias_, imu_accel_bias_,
                                 imu_gyro_bias_,  imu_gyro_bias_,  imu_gyro_bias_).finished());

        // Prior on velocity and bias at bootstrap
        velocityPriorNoise_   = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
        // Fallback velocity random walk — used when IMU factor is unavailable
        // Sigma = 0.5 m/s per keyframe; loosen if vehicle accelerates quickly
        velocityBetweenNoise_ = gtsam::noiseModel::Isotropic::Sigma(3, 0.5);
        biasPriorNoise_       = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << 0.1, 0.1, 0.1, 0.01, 0.01, 0.01).finished());

        // DVL velocity measurement noise
        dvlNoise_ = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(3) << dvl_noise_x_, dvl_noise_y_, dvl_noise_z_).finished());

        // AHRS attitude noise — 2D (on Unit3 tangent space), constrains roll+pitch only
        // AttitudeFactor leaves yaw free; GICP handles yaw via scan matching
        ahrsNoise_ = gtsam::noiseModel::Isotropic::Sigma(2, ahrs_noise_rp_);

        // Accelerometer gravity prior noise — same tangent-space dimension as AHRS
        accelGravityNoise_ = gtsam::noiseModel::Isotropic::Sigma(2, accel_gravity_noise_);

        // Depth factor noise — 1D, scalar sigma in metres
        depthNoise_ = gtsam::noiseModel::Isotropic::Sigma(1, depth_noise_);
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
                     float ekf_z = 0.0f,
                     double stamp_sec = 0.0)
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
        // Enforce minimum time between keyframes so sensors (DVL) have time to accumulate
        if (stamp_sec > 0.0 && (stamp_sec - last_kf_time_) < kf_min_dt_) return;
        last_kf_time_ = stamp_sec;

        // Always lock gtsam_mutex_ BEFORE kf_mutex_
        std::lock_guard<std::mutex> gtsam_lock(gtsam_mutex_);
        std::lock_guard<std::mutex> kf_lock   (kf_mutex_);

        int current_id = static_cast<int>(keyframes_.size());
        gtsam::Pose3 current_gtsam_pose = matrix2Pose3(current_pose);

        if (current_id == 0) {
            // ── Bootstrap: seed initial orientation from AHRS if available ────
            // This ensures IMU gravity is resolved onto the correct axes from
            // frame 1, preventing gravity leaking into horizontal position drift.
            if (use_imu_) {
                gtsam::Rot3 ahrs_init;
                bool got_ahrs = false;
                {
                    std::lock_guard<std::mutex> lk(ahrs_mutex_);
                    if (has_ahrs_) { ahrs_init = latest_ahrs_rot_; got_ahrs = true; }
                }
                if (got_ahrs) {
                    // Keep AHRS roll+pitch; yaw comes from GICP (starts at 0)
                    double r = ahrs_init.roll();
                    double p = ahrs_init.pitch();
                    // double p = 0.0;
                    double y = current_gtsam_pose.rotation().yaw();
                    gtsam::Rot3 init_rot = gtsam::Rot3::RzRyRx(r, p, y);
                    current_gtsam_pose = gtsam::Pose3(init_rot, current_gtsam_pose.translation());
                    if(debug){
                        RCLCPP_INFO(get_logger(),
                            "[Bootstrap] AHRS seed: roll=%.2f°  pitch=%.2f°  yaw=%.2f°",
                            r * 180.0 / M_PI, p * 180.0 / M_PI, y * 180.0 / M_PI);
                    }
                } else {
                    RCLCPP_WARN(get_logger(),
                        "[Bootstrap] No AHRS data yet — X(0) starts with identity roll/pitch. ");
                }
            }

            // ── Bootstrap: priors on pose, velocity, bias ─────────────────────
            gtSAMgraph_.add(gtsam::PriorFactor<gtsam::Pose3>(
                X(0), current_gtsam_pose, priorNoise_));
            initialEstimates_.insert(X(0), current_gtsam_pose);

            if (use_imu_ || use_dvl_) {
                gtsam::Vector3 init_vel = gtsam::Vector3::Zero();
                if (use_dvl_) {
                    std::lock_guard<std::mutex> dlk(dvl_mutex_);
                    if (has_dvl_) {
                        // rotate body-frame DVL to world using current pose
                        init_vel = current_pose.block<3,3>(0,0).cast<double>() * latest_dvl_vel_body_;
                    }
                }
                gtSAMgraph_.add(gtsam::PriorFactor<gtsam::Vector3>(
                    V(0), init_vel, velocityPriorNoise_));
                gtSAMgraph_.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(
                    B(0), gtsam::imuBias::ConstantBias(), biasPriorNoise_));
                initialEstimates_.insert(V(0), init_vel);
                initialEstimates_.insert(B(0), gtsam::imuBias::ConstantBias());
                prev_velocity_ = init_vel;
                prev_bias_     = gtsam::imuBias::ConstantBias();
            }
        } else {
            bool used_imu_factor = false;
            if (use_imu_ && preint_ && current_id >= imu_start_kf_) {
                std::lock_guard<std::mutex> ilk(imu_mutex_);
                if (preint_->deltaTij() > 0.01) {
                    // IMU factor replaces the pose BetweenFactor
                    gtSAMgraph_.add(gtsam::ImuFactor(
                        X(current_id-1), V(current_id-1),
                        X(current_id),   V(current_id),
                        B(current_id-1), *preint_));
                    if (debug) {
                        gtsam::Vector3 dv = preint_->deltaVij();
                        RCLCPP_INFO(get_logger(),
                            "[IMU factor] kf=%d  dt=%.3fs  "
                            "deltaVij=[%.4f, %.4f, %.4f] m/s  "
                            "prev_vel=[%.4f, %.4f, %.4f] m/s",
                            current_id, preint_->deltaTij(),
                            dv.x(), dv.y(), dv.z(),
                            prev_velocity_.x(), prev_velocity_.y(), prev_velocity_.z());
                    }
                    used_imu_factor = true;
                }
            } else if (use_imu_ && current_id < imu_start_kf_) {
                if (debug) {
                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                        "[IMU warmup] kf=%d/%d — using GICP odometry until pose is reliable",
                        current_id, imu_start_kf_);
                }
            }

            gtsam::Pose3 prev_gtsam = matrix2Pose3(keyframes_.back().pose);
            gtsam::Pose3 relative   = prev_gtsam.between(current_gtsam_pose);
            gtSAMgraph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
                X(current_id-1), X(current_id), relative, odomNoise_));
            if (debug) {
                gtsam::Vector3 dt = relative.translation();
                gtsam::Vector3 rpy = relative.rotation().rpy();
                RCLCPP_INFO(get_logger(),
                    "[VGICP factor] kf=%d  dt=[%.4f, %.4f, %.4f] m  drpy=[%.3f, %.3f, %.3f] deg",
                    current_id, dt.x(), dt.y(), dt.z(),
                    rpy(0)*180.0/M_PI, rpy(1)*180.0/M_PI, rpy(2)*180.0/M_PI);
            }

            // Also add IMU factor if available — they work together
            if (use_imu_ && preint_ && current_id >= imu_start_kf_) {
                std::lock_guard<std::mutex> ilk(imu_mutex_);
                if (preint_->deltaTij() > 0.01) {
                    gtSAMgraph_.add(gtsam::ImuFactor(
                        X(current_id-1), V(current_id-1),
                        X(current_id),   V(current_id),
                        B(current_id-1), *preint_));
                    used_imu_factor = true;
                }
            }
            // IMU factor is absent. Without this, the linear system is singular.
            if (use_imu_ || use_dvl_) {
                gtSAMgraph_.add(gtsam::BetweenFactor<gtsam::Vector3>(
                    V(current_id-1), V(current_id),
                    gtsam::Vector3::Zero(), velocityBetweenNoise_));
            }

            if (use_imu_ || use_dvl_) {
                // Bias random walk
                gtSAMgraph_.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(
                    B(current_id-1), B(current_id),
                    gtsam::imuBias::ConstantBias(), biasBetweenNoise_));

                initialEstimates_.insert(B(current_id), prev_bias_);
            }

            if (used_imu_factor) {
                gtsam::NavState prop = preint_->predict(
                    gtsam::NavState(
                        matrix2Pose3(keyframes_.back().pose),  // pose at i-1
                        prev_velocity_),                        // velocity at i-1
                    prev_bias_);
                initialEstimates_.insert(X(current_id), prop.pose());      // IMU-propagated pose
                initialEstimates_.insert(V(current_id), prop.velocity());  // IMU-propagated velocity
            } else {
                initialEstimates_.insert(X(current_id), current_gtsam_pose);  // fall back to GICP
                if (use_imu_ || use_dvl_) {
                    initialEstimates_.insert(V(current_id), prev_velocity_);
                }
            }

            // ── DVL factors (velocity + translation) ────────────────────────
            if (use_dvl_) {
                // 1) Velocity factor at current keyframe — equation (6) in AQUA-SLAM
                //    Constrains V(current_id) via latest DVL body-frame measurement
                gtsam::Vector3 dvl_snap;
                bool dvl_ok = false;
                {
                    std::lock_guard<std::mutex> dlk(dvl_mutex_);
                    if (has_dvl_) { dvl_snap = latest_dvl_vel_body_; dvl_ok = true; }
                }
                if (dvl_ok) {
                    gtSAMgraph_.add(DvlVelocityFactor(
                        X(current_id), V(current_id), dvl_snap, dvlNoise_));
                    if (debug) {
                        RCLCPP_INFO(get_logger(),
                            "[DVL vel factor] kf=%d  body-vel=[%.4f, %.4f, %.4f] m/s",
                            current_id, dvl_snap.x(), dvl_snap.y(), dvl_snap.z());
                    }
                } else if (debug) {
                    RCLCPP_WARN(get_logger(),
                        "[DVL vel factor] kf=%d  skipped — no DVL measurement yet", current_id);
                }

                // 2) Translation factor between keyframes — equation (8) in AQUA-SLAM
                //    Uses DVL pre-integration accumulated since the previous keyframe.
                //    Only available from keyframe 1 onward.
                if (use_dvl_trans_ && current_id > 0) {
                    Eigen::Vector3d preint_dp;
                    bool preint_ok = false;
                    {
                        std::lock_guard<std::mutex> plk(dvl_preint_mutex_);
                        if (dvl_preint_dt_ > 0.01) {  // at least 10 ms of data
                            preint_dp  = dvl_preint_dp_;
                            preint_ok  = true;
                        }
                    }
                    if (preint_ok) {
                        gtSAMgraph_.add(DvlTranslationFactor(
                            X(current_id - 1), X(current_id),
                            preint_dp, R_ID_, R_DC_, D_pDC_,
                            dvlTransNoise_));
                        if (debug) {
                            RCLCPP_INFO(get_logger(),
                                "[DVL trans factor] kf=%d  preint_dp=[%.4f, %.4f, %.4f] m",
                                current_id,
                                preint_dp.x(), preint_dp.y(), preint_dp.z());
                        }
                    } else if (debug) {
                        RCLCPP_WARN(get_logger(),
                            "[DVL trans factor] kf=%d  skipped — insufficient pre-integration dt",
                            current_id);
                    }
                }

                // Reset DVL pre-integration for next keyframe interval
                {
                    std::lock_guard<std::mutex> plk(dvl_preint_mutex_);
                    dvl_preint_dR_ = Eigen::Matrix3d::Identity();
                    dvl_preint_dp_ = Eigen::Vector3d::Zero();
                    dvl_preint_dt_ = 0.0;
                }
            }

            // AHRS attitude factor — constrains roll+pitch only (2 DoF via gravity direction)
            // Prevents gyro-bias-induced tilt from leaking gravity onto horizontal axes
            if (use_ahrs_) {
                gtsam::Rot3 ahrs_snap;
                bool ahrs_ok = false;
                {
                    std::lock_guard<std::mutex> lk(ahrs_mutex_);
                    if (has_ahrs_) { ahrs_snap = latest_ahrs_rot_; ahrs_ok = true; }
                }
                if (ahrs_ok) {
                    // Constraint: R_wb * bMeasured == nRef
                    // nRef    = gravity direction in world   = (0,0,1) for Z-down
                    // bMeasured = gravity direction in body  = R_wb^T * (0,0,1)
                    gtsam::Unit3 g_body(ahrs_snap.transpose() * gtsam::Vector3(0, 0, 1));
                    gtSAMgraph_.add(gtsam::Pose3AttitudeFactor(
                        X(current_id),
                        gtsam::Unit3(0, 0, 1),  // nZ    — gravity direction in world (Z-down)
                        ahrsNoise_,              // noise model
                        g_body));                // bRef  — gravity direction in body frame
                }
            }

            // Accelerometer gravity prior (ORB-SLAM3 style) — mean accel over the
            // keyframe interval approximates gravity direction when motion is slow.
            // A magnitude check rejects windows dominated by dynamic acceleration.
            if (use_accel_gravity_ && !use_ahrs_) {
                gtsam::Vector3 g_mean;
                bool g_ok = false;
                {
                    std::lock_guard<std::mutex> ilk(imu_mutex_);
                    if (accel_count_ > 0) {
                        g_mean = accel_sum_ / static_cast<double>(accel_count_);
                        double g_norm = g_mean.norm();
                        // Accept only when mean magnitude is within 20% of g
                        if (std::abs(g_norm - imu_gravity_) < 0.2 * imu_gravity_) {
                            g_mean /= g_norm;
                            g_ok = true;
                        }
                    }
                    accel_sum_   = gtsam::Vector3::Zero();
                    accel_count_ = 0;
                }
                if (g_ok) {
                    gtSAMgraph_.add(gtsam::Pose3AttitudeFactor(
                        X(current_id),
                        gtsam::Unit3(0, 0, 1),    // gravity in world (Z-down)
                        accelGravityNoise_,
                        gtsam::Unit3(g_mean)));   // mean gravity direction in body frame
                }
            }

            // Depth factor — constrains Z translation to barometric/pressure depth
            if (use_depth_) {
                double depth_snap;
                bool depth_ok = false;
                {
                    std::lock_guard<std::mutex> lk(depth_mutex_);
                    if (has_depth_) { depth_snap = latest_depth_z_; depth_ok = true; }
                }
                if (depth_ok) {
                    gtSAMgraph_.add(DepthFactor(X(current_id), depth_snap, depthNoise_));
                    if (debug) {
                        RCLCPP_INFO(get_logger(),
                            "[Depth factor] kf=%d  z=%.4f m", current_id, depth_snap);
                    }
                } else if (debug) {
                    RCLCPP_WARN(get_logger(),
                        "[Depth factor] kf=%d  skipped — no depth measurement yet", current_id);
                }
            }

            // ── Visual odometry factor ───────────────────────────────────────
            if (use_vo_ && current_id > 0) {
                Eigen::Matrix4f vo_snap;
                bool vo_ok = false;
                bool reset_flag = false;
                {
                    std::lock_guard<std::mutex> lk(vo_mutex_);
                    reset_flag = vo_reset_pending_;
                    vo_reset_pending_ = false;
                    if (has_vo_) { vo_snap = latest_vo_pose_; vo_ok = true; }
                }
                if (reset_flag) {
                    if (debug) {
                        RCLCPP_WARN(get_logger(),
                            "[VO factor] kf=%d  skipped — VO reset between keyframes", current_id);
                    }
                    prev_vo_pose_valid_ = false;
                } else if (vo_ok && prev_vo_pose_valid_) {
                    // Relative delta in VO frame — frame-origin-independent
                    Eigen::Matrix4f vo_delta = prev_vo_pose_.inverse() * vo_snap;
                    float delta_t = vo_delta.block<3,1>(0,3).norm();
                    if (delta_t > static_cast<float>(vo_max_delta_)) {
                        if (debug) {
                            RCLCPP_WARN(get_logger(),
                                "[VO factor] kf=%d  skipped — delta too large (%.2fm)", current_id, delta_t);
                        }
                        prev_vo_pose_valid_ = false;
                    } else {
                        gtsam::Pose3 relative = matrix2Pose3(vo_delta);
                        gtSAMgraph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
                            X(current_id-1), X(current_id), relative, voNoise_));
                        if (debug) {
                            RCLCPP_INFO(get_logger(),
                                "[VO factor] kf=%d  dt=[%.3f, %.3f, %.3f] m",
                                current_id,
                                vo_delta(0,3), vo_delta(1,3), vo_delta(2,3));
                        }
                    }
                } else if (!prev_vo_pose_valid_ && vo_ok) {
                    if (debug) {
                        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                            "[VO factor] waiting for two consecutive valid VO poses");
                    }
                }
                if (vo_ok && !reset_flag) {
                    prev_vo_pose_       = vo_snap;
                    prev_vo_pose_valid_ = true;
                }
            }
        }
        isam_->update(gtSAMgraph_, initialEstimates_);
        isam_->update();
        gtSAMgraph_.resize(0);
        initialEstimates_.clear();

        gtsam::Pose3    optimised_pose = isam_->calculateEstimate<gtsam::Pose3>(X(current_id));
        Eigen::Matrix4f optimised_mat  = pose32Matrix(optimised_pose);
        if (ekf_z_ && loop_ekf_z_) optimised_mat(2, 3) = ekf_z;

        // Read back velocity and bias, reset preintegrator
        if (use_imu_ || use_dvl_) {
            prev_velocity_ = isam_->calculateEstimate<gtsam::Vector3>(V(current_id));
            prev_bias_     = isam_->calculateEstimate<gtsam::imuBias::ConstantBias>(B(current_id));
            if (use_imu_ && preint_) {
                std::lock_guard<std::mutex> ilk(imu_mutex_);
                preint_ = std::make_shared<gtsam::PreintegratedImuMeasurements>(
                    preint_->params(), prev_bias_);
            }
        }

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

        if (debug) { RCLCPP_INFO(get_logger(), "Keyframe %d added.", current_id); }
    }

    // ── Loop closure thread ───────────────────────────────────────────────────
    void loopClosureThread()
    {
        while (rclcpp::ok() && use_lc_) {
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
        ds_filter.setLeafSize(map_res_, map_res_, map_res_);
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

        // // Initial guess: expected transform from latest to history in world frame.
        // // Using identity assumes no drift; using the pose difference handles larger drift
        // // and keeps VGICP in the correct basin of attraction.
        // Eigen::Matrix4f lc_initial_guess = history_pose_world * latest_pose_world.inverse();
        Eigen::Matrix4f lc_initial_guess = Eigen::Matrix4f::Identity();
        if (lc_use_ndt_) {
            ndt_lc_.setInputTarget(history_ds);
            ndt_lc_.setInputSource(latest_cloud_world);
            ndt_lc_.align(aligned, lc_initial_guess);
            lc_converged = ndt_lc_.hasConverged();
            score        = ndt_lc_.getFitnessScore();
            correction   = ndt_lc_.getFinalTransformation();
        } else {
            vgicp_lc_.setInputSource(latest_cloud_world);
            vgicp_lc_.setInputTarget(history_ds);
            vgicp_lc_.align(aligned, lc_initial_guess);
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
        if (correction_dist > static_cast<float>(lc_max_correction_dist_) ||
            correction_angle > static_cast<float>(lc_max_correction_angle_)) {
            RCLCPP_WARN(get_logger(),
                "Loop closure refused: correction too large (t=%.2fm angle=%.1fdeg) "
                "— likely wrong minimum",
                correction_dist, correction_angle);
            return;
        }

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
        // vg.setLeafSize(0.2f, 0.2f, 0.2f);
        vg.setLeafSize(map_res_, map_res_, map_res_);
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

    // ── IMU callback — accumulates preintegration between keyframes ───────────
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        double t = rclcpp::Time(msg->header.stamp).seconds();
        std::lock_guard<std::mutex> lk(imu_mutex_);
        if (!has_imu_first_) {
            imu_last_time_ = t;
            has_imu_first_ = true;
            return;
        }
        double dt = t - imu_last_time_;
        if (dt <= 0.0 || dt > 0.5) { imu_last_time_ = t; return; }

        // Subtract static biases measured at rest so the preintegrator is accurate
        // from keyframe 0. GTSAM's bias state corrects any remaining residual.
        gtsam::Vector3 accel = R_imu2base_ * gtsam::Vector3(
            msg->linear_acceleration.x - imu_accel_bias_x_,
            msg->linear_acceleration.y - imu_accel_bias_y_,
            msg->linear_acceleration.z - imu_accel_bias_z_);
        gtsam::Vector3 gyro = R_imu2base_ * gtsam::Vector3(
            msg->angular_velocity.x - imu_gyro_bias_x_,
            msg->angular_velocity.y - imu_gyro_bias_y_,
            msg->angular_velocity.z - imu_gyro_bias_z_);

        // Reject spikes — corrupted IMU samples poison preintegration
        if (accel.norm() > imu_max_accel_ || gyro.norm() > imu_max_gyro_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "[IMU cb] rejected spike — accel=%.1f m/s^2  gyro=%.1f rad/s",
                accel.norm(), gyro.norm());
            imu_last_time_ = t;
            return;
        }

        preint_->integrateMeasurement(accel, gyro, dt);
        if (scan_preint_) scan_preint_->integrateMeasurement(accel, gyro, dt);
        imu_last_time_ = t;

        // ── Accelerometer gravity accumulation (ORB-SLAM3 style) ─────────────
        // Sum raw accel samples between keyframes. AddKeyFrame computes the mean,
        // which approximates the gravity direction when motion is slow.
        // imu_mutex_ is already held here.
        if (use_accel_gravity_) {
            accel_sum_   += accel;
            accel_count_ += 1;
        }

        // ── Accumulate gyro rotation for DVL pre-integration ─────────────────
        // Between DVL pings the vehicle rotates. We track this so each DVL
        // velocity sample is correctly rotated before being summed into ΔDi_p̄.
        // ΔR̂_IiIk = Π Exp(ω * dt)  — we use small-angle: Exp(ω*dt) ≈ I + [ω*dt]×
        if (use_dvl_) {
            Eigen::Vector3d w(gyro.x(), gyro.y(), gyro.z());
            double angle = w.norm() * dt;
            Eigen::Matrix3d dR_step;
            if (angle > 1e-8) {
                Eigen::AngleAxisd aa(angle, w.normalized());
                dR_step = aa.toRotationMatrix();
            } else {
                // Small angle: Exp(ω*dt) ≈ I + [ω*dt]×
                Eigen::Matrix3d skew;
                skew <<       0.0, -w.z()*dt,  w.y()*dt,
                         w.z()*dt,       0.0, -w.x()*dt,
                        -w.y()*dt,  w.x()*dt,       0.0;
                dR_step = Eigen::Matrix3d::Identity() + skew;
            }
            {
                std::lock_guard<std::mutex> plk(dvl_preint_mutex_);
                dvl_preint_dR_ = dvl_preint_dR_ * dR_step;
            }
            {
                std::lock_guard<std::mutex> dlk(dvl_mutex_);
                scan_dvl_dR_ = scan_dvl_dR_ * dR_step;
            }
        }
    }

    // ── DVL callback — latches latest body-frame velocity ────────────────────
    void dvlCallback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
    {
        gtsam::Vector3 vel(msg->twist.twist.linear.x,
                           msg->twist.twist.linear.y,
                           msg->twist.twist.linear.z);

        // Lever arm correction: v_base = v_dvl - ω × r_base2dvl
        // The DVL measures velocity at dvl_link; GTSAM tracks base_link.
        // When rotating, the two differ by ω × r.
        if (r_base2dvl_.norm() > 1e-6) {
            gtsam::Vector3 omega(msg->twist.twist.angular.x,
                                 msg->twist.twist.angular.y,
                                 msg->twist.twist.angular.z);
            gtsam::Vector3 lever_vel = omega.cross(r_base2dvl_);
            if(debug){
                RCLCPP_DEBUG(get_logger(),
                    "[DVL lever] omega=[%.4f,%.4f,%.4f] r=[%.3f,%.3f,%.3f] correction=[%.4f,%.4f,%.4f]",
                    omega.x(), omega.y(), omega.z(),
                    r_base2dvl_.x(), r_base2dvl_.y(), r_base2dvl_.z(),
                    lever_vel.x(), lever_vel.y(), lever_vel.z());
            }
            vel -= lever_vel;
        }
        if(debug){
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                "[DVL cb] body-frame vel (corrected)  x=%.4f  y=%.4f  z=%.4f  m/s",
                vel.x(), vel.y(), vel.z());
        }

        // Reject implausibly large readings (beam failures / dropouts)
        if (vel.norm() > dvl_max_vel_) {
            if(debug){
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                    "[DVL cb] rejected — speed %.2f m/s exceeds max_vel %.2f m/s",
                    vel.norm(), dvl_max_vel_);
            }
            return;
        }

        // Update velocity latch
        {
            std::lock_guard<std::mutex> lk(dvl_mutex_);
            latest_dvl_vel_body_ = vel;
            has_dvl_ = true;
        }

        // ── DVL pre-integration (equation 8, AQUA-SLAM) ──────────────────────
        // Each time a DVL ping arrives, we advance the pre-integrated position
        // using the current accumulated gyro rotation and this velocity sample.
        // The gyro rotation dvl_preint_dR_ is updated in imuCallback below.
        //
        // ΔDi_p̄ += ΔR̂_IiIk * R_ID * Di_v * Δt_dvl
        // We use the DVL ping interval as Δt (typically 0.2 s at 5 Hz).
        // For a more accurate integration, imuCallback accumulates dR between pings.
        static rclcpp::Time last_dvl_time{0, 0, RCL_ROS_TIME};
        rclcpp::Time now = msg->header.stamp;
        double dt_dvl = 0.0;
        if (last_dvl_time.nanoseconds() > 0) {
            dt_dvl = (now - last_dvl_time).seconds();
        }
        last_dvl_time = now;

        if (dt_dvl > 0.001 && dt_dvl < 1.0) {  // sanity: between 1ms and 1s
            Eigen::Vector3d v_dvl(vel.x(), vel.y(), vel.z());
            {
                std::lock_guard<std::mutex> plk(dvl_preint_mutex_);
                dvl_preint_dp_ += dvl_preint_dR_ * R_ID_ * v_dvl * dt_dvl;
                dvl_preint_dt_ += dt_dvl;
                dvl_preint_dR_ = Eigen::Matrix3d::Identity();
            }
            {
                // Rotate DVL sample into the scan-start body frame before integrating,
                // same as dvl_preint_dp_ does — closes the gap with EKF quality.
                std::lock_guard<std::mutex> dlk(dvl_mutex_);
                Eigen::Vector3f v_rotated = scan_dvl_dR_.cast<float>() * v_dvl.cast<float>();
                scan_dvl_dp_ += v_rotated * static_cast<float>(dt_dvl);
                scan_dvl_valid_ = true;
            }
        }
    }

    // ── EKF callback ──────────────────────────────────────────────────────────
    void voCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
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

        // Reject invalid quaternion — zero-norm → NaN after normalization → SIGFPE
        if (q.norm() < 1e-6f) {
            std::lock_guard<std::mutex> lock(vo_mutex_);
            vo_reset_pending_ = true;
            if(debug){
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "[VO] invalid quaternion (norm=%.6f) — treating as reset", q.norm());
            }
            return;
        }

        // Detect reset: VO publishes near-zero position when tracking fails
        if (t.norm() < static_cast<float>(vo_reset_thresh_)) {
            std::lock_guard<std::mutex> lock(vo_mutex_);
            vo_reset_pending_ = true;
            if(debug){
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                    "[VO] tracking lost / reset detected (pos norm=%.4f)", t.norm());
            }
            return;
        }

        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        pose.block<3,3>(0,0) = q.normalized().toRotationMatrix();
        pose.block<3,1>(0,3) = t;

        std::lock_guard<std::mutex> lock(vo_mutex_);
        latest_vo_pose_ = pose;
        has_vo_ = true;
    }

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
        if (use_ekf_) {
            if (!has_ekf_) {
                if (debug) { RCLCPP_WARN_ONCE(get_logger(), "Waiting for first EKF message..."); }
                return;
            }
            {
                std::lock_guard<std::mutex> lock(ekf_mutex_);
                if (ekf_msg_count_ < min_ekf_msgs_) {
                    if (debug) {
                        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                            "Waiting for EKF to converge (%d / %d messages)",
                            ekf_msg_count_.load(), min_ekf_msgs_);
                    }
                    return;
                }
            }
            {
                std::lock_guard<std::mutex> lock(ekf_mutex_);
                double dt = std::abs((rclcpp::Time(msg->header.stamp) - rclcpp::Time(latest_ekf_stamp_)).seconds());
                if (dt > ekf_max_age_) {
                    if (debug) {
                        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                            "EKF message is %.3fs old (threshold %.3fs) — skipping scan", dt, ekf_max_age_);
                    }
                    return;
                }
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

        if (filtered->empty()) return;  // nothing to match against — avoids VGICP SIGFPE

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
            Eigen::Matrix4f bootstrap_pose = use_ekf_ ? current_ekf_pose : Eigen::Matrix4f::Identity();
            {
                std::lock_guard<std::mutex> pose_lock(pose_mutex_);
                global_pose_ = bootstrap_pose;
            }
            if (use_ekf_) prev_ekf_pose_ = current_ekf_pose;
            AddKeyFrame(bootstrap_pose, filtered, bootstrap_pose(2, 3),
                        rclcpp::Time(msg->header.stamp).seconds());
            return;
        }

        // 6. Build initial guess
        Eigen::Matrix4f current_global;
        {
            std::lock_guard<std::mutex> pose_lock(pose_mutex_);
            current_global = global_pose_;
        }
        Eigen::Matrix4f initial_guess;
        if (use_ekf_) {
            Eigen::Matrix4f ekf_delta = prev_ekf_pose_.inverse() * current_ekf_pose;
            initial_guess = current_global * ekf_delta;
        } else if (use_dvl_) {
            bool dvl_ok;
            Eigen::Vector3f dvl_integrated;
            {
                std::lock_guard<std::mutex> dlk(dvl_mutex_);
                dvl_ok       = scan_dvl_valid_;
                dvl_integrated = scan_dvl_dp_;
            }

            if (dvl_ok) {
                // Gyro rotation (deltaRij uses gyro only, NOT accelerometer) + DVL translation
                gtsam::Rot3 delta_R;
                {
                    std::lock_guard<std::mutex> ilk(imu_mutex_);
                    delta_R = scan_preint_->deltaRij();
                }
                Eigen::Matrix3f R_wb = current_global.block<3,3>(0,0);
                Eigen::Vector3f t_delta = R_wb * dvl_integrated;

                Eigen::Matrix4f scan_delta = Eigen::Matrix4f::Identity();
                scan_delta.block<3,3>(0,0) = delta_R.matrix().cast<float>();
                scan_delta.block<3,1>(0,3) = t_delta;
                initial_guess = current_global * scan_delta;
            } else {
                // DVL dead — gyro rotation only, no translation prediction
                gtsam::Rot3 delta_R;
                {
                    std::lock_guard<std::mutex> ilk(imu_mutex_);
                    delta_R = scan_preint_->deltaRij();
                }
                Eigen::Matrix4f scan_delta = Eigen::Matrix4f::Identity();
                scan_delta.block<3,3>(0,0) = delta_R.matrix().cast<float>();
                initial_guess = current_global * scan_delta;
            }
        } else if (has_prev_scan_) {
            // Constant velocity fallback — IMU not yet running
            Eigen::Matrix4f delta = prev_scan_pose_.inverse() * current_global;
            initial_guess = current_global * delta;
        } else {
            initial_guess = current_global;  // first scan — zero-motion
        }

        // Override Z with depth sensor when EKF is off — more reliable than DVL Z integration
        if (!use_ekf_ && use_depth_) {
            double depth_snap;
            bool depth_ok = false;
            {
                std::lock_guard<std::mutex> lk(depth_mutex_);
                if (has_depth_) { depth_snap = latest_depth_z_; depth_ok = true; }
            }
            if (depth_ok) initial_guess(2, 3) = static_cast<float>(depth_snap);
        }

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

        // hasConverged() must be called first — querying score/transform before it
        // resets fast_gicp's internal state and causes it to return false.
        bool converged = vgicp_.hasConverged();

        Eigen::Matrix4f result        = vgicp_.getFinalTransformation();
        double          gicp_score    = vgicp_.getFitnessScore();
        Eigen::Matrix4f diff          = initial_guess.inverse() * result;
        float correction_dist         = diff.block<3,1>(0,3).norm();
        float correction_angle        = Eigen::AngleAxisf(
            Eigen::Matrix3f(diff.block<3,3>(0,0))).angle() * 180.0f / M_PI;

        bool gicp_rejected = converged && (
            (gicp_fitness_score_ > 0.0 && gicp_score > gicp_fitness_score_) ||
            correction_dist  > static_cast<float>(gicp_max_correction_dist_) ||
            correction_angle > static_cast<float>(gicp_max_correction_angle_));

        if (debug && converged) {
            RCLCPP_INFO(get_logger(),
                "GICP | score: %.4f | correction: t=%.2fm angle=%.1fdeg | src: %zu | tgt: %zu",
                gicp_score, correction_dist, correction_angle,
                filtered->size(), map_snapshot->size());
        }

        if (converged && !gicp_rejected) {
            lost_frames_ = 0;

            {
                std::lock_guard<std::mutex> pose_lock(pose_mutex_);
                if (use_ekf_ && ekf_z_) result(2, 3) = current_ekf_pose(2, 3);
                global_pose_   = result;
                current_global = result;
            }
            double score = vgicp_.getFitnessScore();
            publishOdometry(msg->header, score, false);
            float kf_z = use_ekf_ ? current_ekf_pose(2, 3) : current_global(2, 3);
            AddKeyFrame(current_global, filtered, kf_z,
                        rclcpp::Time(msg->header.stamp).seconds());

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

            if (use_ekf_) prev_ekf_pose_ = current_ekf_pose;
            prev_scan_pose_ = current_global;
            has_prev_scan_  = true;
            if (scan_preint_) {
                std::lock_guard<std::mutex> ilk(imu_mutex_);
                scan_preint_ = std::make_shared<gtsam::PreintegratedImuMeasurements>(
                    scan_preint_->params(), prev_bias_);
            }
            {
                std::lock_guard<std::mutex> dlk(dvl_mutex_);
                scan_dvl_dp_    = Eigen::Vector3f::Zero();
                scan_dvl_dR_    = Eigen::Matrix3d::Identity();
                scan_dvl_valid_ = false;
            }

        } else {
            if (gicp_rejected) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                    "GICP rejected — score=%.4f (max %.4f)  t=%.2fm (max %.2f)  angle=%.1fdeg (max %.1f)",
                    gicp_score,       gicp_fitness_score_,
                    correction_dist,  gicp_max_correction_dist_,
                    correction_angle, gicp_max_correction_angle_);
            } else if (debug) {
                RCLCPP_WARN(get_logger(), "VGICP did not converge (lost=%d)",
                    lost_frames_);
            }
            if (lost_frames_ < max_lost_frames) {
                ++lost_frames_;
                {
                    std::lock_guard<std::mutex> pose_lock(pose_mutex_);
                    global_pose_ = initial_guess;
                    if (use_ekf_) prev_ekf_pose_ = current_ekf_pose;
                }
                if (scan_preint_) {
                    std::lock_guard<std::mutex> ilk(imu_mutex_);
                    scan_preint_ = std::make_shared<gtsam::PreintegratedImuMeasurements>(
                        scan_preint_->params(), prev_bias_);
                }
                {
                    std::lock_guard<std::mutex> dlk(dvl_mutex_);
                    scan_dvl_dp_    = Eigen::Vector3f::Zero();
                    scan_dvl_dR_    = Eigen::Matrix3d::Identity();
                    scan_dvl_valid_ = false;
                }
                prev_scan_pose_ = initial_guess;
                has_prev_scan_  = true;
                // Directly push the current scan into the local map at the
                // dead-reckoning pose.  AddKeyFrame is not used here because its
                // distance threshold (0.5 m) silently returns without updating when
                // frames arrive at 6 Hz — exactly the condition that freezes the
                // target and causes the score to keep exploding.
                {
                    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_in_odom(
                        new pcl::PointCloud<pcl::PointXYZ>);
                    pcl::transformPointCloud(*filtered, *scan_in_odom, initial_guess);
                    std::lock_guard<std::mutex> kf_lk(kf_mutex_);
                    *local_map_ += *scan_in_odom;
                    pcl::PointCloud<pcl::PointXYZ>::Ptr ds(
                        new pcl::PointCloud<pcl::PointXYZ>);
                    map_filter_.setInputCloud(local_map_);
                    map_filter_.filter(*ds);
                    local_map_ = ds;
                }
            } else {
                RCLCPP_WARN(get_logger(), "Tracking lost — restarting SLAM.");
                lost_frames_ = 0;

                {
                    std::lock_guard<std::mutex> pose_lock(pose_mutex_);
                    global_pose_ = use_ekf_ ? current_ekf_pose : initial_guess;
                }
                if (use_ekf_) prev_ekf_pose_ = current_ekf_pose;

                {
                    std::lock_guard<std::mutex> gtsam_lock(gtsam_mutex_);
                    std::lock_guard<std::mutex> kf_lock   (kf_mutex_);
                    slam_generation_++;
                    initGTSAM();
                    keyframes_.clear();
                    local_map_->clear();
                }
                if (use_imu_ && preint_) {
                    std::lock_guard<std::mutex> ilk(imu_mutex_);
                    prev_bias_     = gtsam::imuBias::ConstantBias();
                    prev_velocity_ = gtsam::Vector3::Zero();
                    preint_ = std::make_shared<gtsam::PreintegratedImuMeasurements>(
                    preint_->params(), prev_bias_);
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

        if (publish_tf_ && tf_broadcaster_) {
            geometry_msgs::msg::TransformStamped tf_msg;
            tf_msg.header.stamp    = header.stamp;
            tf_msg.header.frame_id = odom_frame_;
            tf_msg.child_frame_id  = base_frame_;
            tf_msg.transform.translation.x = t.x();
            tf_msg.transform.translation.y = t.y();
            tf_msg.transform.translation.z = t.z();
            tf_msg.transform.rotation.x    = q.x();
            tf_msg.transform.rotation.y    = q.y();
            tf_msg.transform.rotation.z    = q.z();
            tf_msg.transform.rotation.w    = q.w();
            tf_broadcaster_->sendTransform(tf_msg);
        }
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
    Eigen::Matrix4f  sonar2base_, base2sonar_, ned_transform_;
    Eigen::Matrix3d  R_imu2base_;
    gtsam::Vector3   r_base2dvl_{gtsam::Vector3::Zero()};  // base_link → dvl_link in body frame [m]

    // ── Config ────────────────────────────────────────────────────────────────
    std::string odom_frame_, base_frame_;
    bool        is_ned_{false};

    int    submap_size_{20};
    double map_res_{0.1};
    double kf_dist_thresh_{0.5}, kf_angle_thresh_{10.0}, kf_min_dt_{0.3};
    double last_kf_time_{0.0};

    double lc_search_radius_{10.0}, lc_fitness_score_{0.3};
    double lc_max_correction_dist_{3.0}, lc_max_correction_angle_{30.0};
    int    lc_history_gap_{10}, lc_submap_size_{7};
    bool   lc_use_ndt_{false};
    bool use_lc_{true};

    // GICP sanity check thresholds
    double gicp_max_correction_dist_{1.0};   // meters
    double gicp_max_correction_angle_{15.0}; // degrees
    double gicp_fitness_score_{0.0};         // reject scan if VGICP score exceeds this (0 = disabled)

    int lost_frames_{0};
    int max_lost_frames{50};
    int map_pub_count_{0};

    bool filter_outliers;
    bool filter_radius_outliers_;
    bool filter_intensity;
    double min_intensity;
    bool use_ekf_{true};
    bool ekf_z_{false};
    bool debug{false};
    bool publish_tf_{false};
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Visual odometry
    bool   use_vo_{false};
    double vo_max_delta_{2.0};
    double vo_reset_thresh_{0.05};
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vo_sub_;
    std::mutex          vo_mutex_;
    Eigen::Matrix4f     latest_vo_pose_{Eigen::Matrix4f::Identity()};
    Eigen::Matrix4f     prev_vo_pose_{Eigen::Matrix4f::Identity()};
    bool                has_vo_{false};
    bool                prev_vo_pose_valid_{false};
    bool                vo_reset_pending_{false};
    gtsam::SharedNoiseModel voNoise_;
    Eigen::Matrix4f prev_scan_pose_{Eigen::Matrix4f::Identity()};
    bool            has_prev_scan_{false};
    bool loop_ekf_z_{false};

    double odom_noise_roll_{0.1},  odom_noise_pitch_{0.1}, odom_noise_yaw_{0.3};
    double odom_noise_x_{0.5},     odom_noise_y_{0.5},     odom_noise_z_{0.3};

    double lc_noise_roll_{0.01},   lc_noise_pitch_{0.01},  lc_noise_yaw_{0.01};
    double lc_noise_x_{0.05},      lc_noise_y_{0.05},      lc_noise_z_{0.05};
    double lc_huber_k_{1.0};

    // ── IMU / DVL ─────────────────────────────────────────────────────────────
    bool   use_imu_{false}, use_dvl_{false}, use_dvl_trans_{false};
    double dvl_max_vel_{3.0};
    double imu_max_accel_{50.0};
    double imu_max_gyro_{10.0};
    int    imu_start_kf_{0};   // keyframes to wait before activating ImuFactor

    // IMU preintegration
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    std::mutex                                              imu_mutex_;
    std::shared_ptr<gtsam::PreintegratedImuMeasurements>   preint_;
    std::shared_ptr<gtsam::PreintegratedImuMeasurements>   scan_preint_;  // resets each scan for initial-guess prediction
    Eigen::Vector3f   scan_dvl_dp_{Eigen::Vector3f::Zero()};   // integrated DVL translation since last scan
    Eigen::Matrix3d   scan_dvl_dR_{Eigen::Matrix3d::Identity()}; // accumulated rotation since last scan (for DVL integration)
    bool              scan_dvl_valid_{false};
    double imu_last_time_{0.0};
    bool   has_imu_first_{false};

    // DVL velocity latch
    rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr dvl_sub_;
    std::mutex     dvl_mutex_;
    gtsam::Vector3 latest_dvl_vel_body_{gtsam::Vector3::Zero()};
    bool           has_dvl_{false};

    // ── DVL pre-integration buffer ────────────────────────────────────────────
    // Between keyframes i and j we accumulate:
    //   ΔDi_p̄_DiDj = Σ ΔR̂_IiIk * R_ID * Di_v * Δt
    // where ΔR̂_IiIk comes from gyro integration (stored in preint_ dR)
    // and Di_v is the last DVL measurement held constant between pings.
    std::mutex          dvl_preint_mutex_;
    Eigen::Matrix3d     dvl_preint_dR_{Eigen::Matrix3d::Identity()};  // accumulated gyro rotation since last KF
    Eigen::Vector3d     dvl_preint_dp_{Eigen::Vector3d::Zero()};      // accumulated ΔDi_p̄
    double              dvl_preint_dt_{0.0};                          // total integrated time
    bool                dvl_preint_ready_{false};                     // true once at least one full KF interval done

    // DVL-IMU-camera extrinsic calibration (loaded from params)
    Eigen::Matrix3d R_ID_{Eigen::Matrix3d::Identity()};  // IMU frame → DVL frame rotation
    Eigen::Matrix3d R_DC_{Eigen::Matrix3d::Identity()};  // DVL frame → camera frame rotation
    Eigen::Vector3d D_pDC_{Eigen::Vector3d::Zero()};     // translation DVL→camera in DVL frame

    // Translation noise for DVL pre-integration factor
    gtsam::noiseModel::Diagonal::shared_ptr dvlTransNoise_;

    // GTSAM velocity + bias state (only used when use_imu_ || use_dvl_)
    gtsam::Vector3                  prev_velocity_{gtsam::Vector3::Zero()};
    gtsam::imuBias::ConstantBias    prev_bias_;

    // Noise models for IMU / DVL / AHRS
    gtsam::noiseModel::Diagonal::shared_ptr  biasBetweenNoise_;
    gtsam::noiseModel::Isotropic::shared_ptr velocityPriorNoise_;
    gtsam::noiseModel::Isotropic::shared_ptr velocityBetweenNoise_;
    gtsam::noiseModel::Diagonal::shared_ptr  biasPriorNoise_;
    gtsam::noiseModel::Diagonal::shared_ptr  dvlNoise_;
    gtsam::noiseModel::Isotropic::shared_ptr ahrsNoise_;  // 2D — roll+pitch only

    // ── Depth ─────────────────────────────────────────────────────────────────
    bool   use_depth_{false};
    double depth_noise_{0.05};
    bool   has_depth_{false};
    double latest_depth_z_{0.0};
    std::mutex depth_mutex_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr depth_sub_;
    gtsam::noiseModel::Isotropic::shared_ptr depthNoise_;

    // ── AHRS attitude ─────────────────────────────────────────────────────────
    bool   use_ahrs_{false};
    bool   has_ahrs_{false};
    std::mutex ahrs_mutex_;
    gtsam::Rot3 latest_ahrs_rot_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr ahrs_sub_;
    double ahrs_noise_rp_{0.02};   // ~1° sigma for roll and pitch
    double ahrs_noise_yaw_{0.1};   // kept for future full-orientation use

    // ── Accelerometer gravity prior (ORB-SLAM3 style) ────────────────────────
    // Protected by imu_mutex_. Accumulates raw accel samples between keyframes;
    // AddKeyFrame computes the mean, checks |mean| ≈ g, then resets.
    bool           use_accel_gravity_{false};
    double         accel_gravity_noise_{0.1};           // [rad]
    gtsam::Vector3 accel_sum_{gtsam::Vector3::Zero()};  // sum of body-frame accel samples
    int            accel_count_{0};                     // number of samples in sum
    gtsam::noiseModel::Isotropic::shared_ptr accelGravityNoise_;

    // IMU noise parameters (needed in initGTSAM which can run after constructor)
    double imu_accel_noise_{0.05},  imu_gyro_noise_{0.005};
    double imu_accel_bias_{1e-3},   imu_gyro_bias_{1e-3};
    double imu_gravity_{9.81};
    // Static bias offsets subtracted per-sample before GTSAM bias estimation kicks in
    double imu_accel_bias_x_{0.0},  imu_accel_bias_y_{0.0},  imu_accel_bias_z_{0.0};
    double imu_gyro_bias_x_{0.0},   imu_gyro_bias_y_{0.0},   imu_gyro_bias_z_{0.0};

    // DVL noise parameters
    double dvl_noise_x_{0.05}, dvl_noise_y_{0.05}, dvl_noise_z_{0.05};
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