#pragma once

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
#include <pcl/filters/passthrough.h>

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
#include <interfaces/msg/magnetometer.hpp>
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
    float ekf_z{0.0f};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
};

class GicpOdomNode : public rclcpp::Node
{
public:
    explicit GicpOdomNode(const rclcpp::NodeOptions & options);
    ~GicpOdomNode();

private:
    // ── GTSAM initialisation ──────────────────────────────────────────────────
    void initGTSAM();

    // ── Helpers ───────────────────────────────────────────────────────────────
    gtsam::Pose3    matrix2Pose3(const Eigen::Matrix4f & m);
    Eigen::Matrix4f pose32Matrix(const gtsam::Pose3 & p);

    // ── Keyframe management ───────────────────────────────────────────────────
    void AddKeyFrame(const Eigen::Matrix4f & current_pose,
                     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                     float ekf_z = 0.0f,
                     double stamp_sec = 0.0,
                     double gicp_score = 0.0);

    // ── Loop closure thread ───────────────────────────────────────────────────
    void loopClosureThread();

    // ── Loop closure — LIO-SAM style ──────────────────────────────────────────
    void performLoopClosure();

    // ── Submap rebuild — call with kf_mutex_ already held ────────────────────
    void updateSubmap();

    // ── Full map publisher — call with kf_mutex_ already held ────────────────
    void publishFullMap();

    // ── Path publisher — call with kf_mutex_ already held ────────────────────
    void publishPath();

    // ── IMU callback — accumulates preintegration between keyframes ───────────
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

    // ── DVL callback — latches latest body-frame velocity ────────────────────
    void dvlCallback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg);

    // ── VO callback ───────────────────────────────────────────────────────────
    void voCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // ── EKF callback ──────────────────────────────────────────────────────────
    void ekfCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // ── Point cloud callback ───────────────────────────────────────────────────
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    // ── Odometry publisher ────────────────────────────────────────────────────
    void publishOdometry(const std_msgs::msg::Header & header, double fitness_score = 0.1, bool is_global_match = false);

    void publishLoopConstraints(int latest_id, int closest_id);

    // ── ROS interfaces ────────────────────────────────────────────────────────
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr        ekf_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr           odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr     global_map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr     full_map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr     filtered_pc_pub_;
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
    gtsam::noiseModel::Base::shared_ptr     robustLoopNoise_;

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
    double               sensor_max_age_{0.1};
    double               dvl_last_time_{0.0};

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
    bool   filter_range_{false};
    double min_range_{0.0};
    double max_range_{10.0};
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
    bool   vgicp_static_noise_{true};   // true = fixed odomNoise_; false = scale by fitness score
    double vgicp_noise_scale_{10.0};    // multiplier: sigma *= (1 + score * scale)
    double vgicp_skip_score_{0.0};      // skip BetweenFactor entirely if score > this (0 = disabled)
    bool   vgicp_use_huber_{false};     // wrap noise model in Huber robust kernel
    double vgicp_huber_k_{1.345};       // Huber threshold (1.345 = 95% efficiency on Gaussian)

    double lc_noise_roll_{0.01},   lc_noise_pitch_{0.01},  lc_noise_yaw_{0.01};
    double lc_noise_x_{0.05},      lc_noise_y_{0.05},      lc_noise_z_{0.05};
    bool   lc_use_huber_{true};
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

    // AQUA-SLAM gravity initialisation
    boost::shared_ptr<gtsam::PreintegrationParams>  imu_preint_params_;   // shared with preint_ / scan_preint_
    gtsam::Vector3  dirG_accum_{gtsam::Vector3::Zero()};  // running sum for gravity direction estimation
    int             gravity_init_count_{0};
    bool            gravity_initialized_{false};
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
    gtsam::Vector3                  latest_gyro_body_{gtsam::Vector3::Zero()};

    // Noise models for IMU / DVL / AHRS
    gtsam::noiseModel::Diagonal::shared_ptr  biasBetweenNoise_;
    gtsam::noiseModel::Isotropic::shared_ptr velocityPriorNoise_;
    gtsam::noiseModel::Isotropic::shared_ptr velocityBetweenNoise_;
    gtsam::noiseModel::Diagonal::shared_ptr  biasPriorNoise_;
    gtsam::noiseModel::Diagonal::shared_ptr  dvlNoise_;
    gtsam::noiseModel::Isotropic::shared_ptr ahrsNoise_;  // 2D — roll+pitch only

    // ── Magnetometer yaw (initial_guess only) ─────────────────────────────────
    bool   use_mag_yaw_{false};
    double mag_declination_{0.0};
    bool   has_mag_{false};
    float  latest_mag_x_{0.0f}, latest_mag_y_{0.0f}, latest_mag_z_{0.0f};
    double mag_yaw_origin_{0.0};
    bool   mag_yaw_origin_set_{false};
    std::mutex mag_mutex_;
    rclcpp::Subscription<interfaces::msg::Magnetometer>::SharedPtr mag_sub_;

    // ── Depth ─────────────────────────────────────────────────────────────────
    bool   use_depth_{false};
    double depth_noise_{0.05};
    bool   has_depth_{false};
    double latest_depth_z_{0.0};
    double depth_origin_{0.0};   // captured at kf-0 when !use_ekf_; relative depth = latest - origin
    std::mutex depth_mutex_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr depth_sub_;
    gtsam::noiseModel::Isotropic::shared_ptr depthNoise_;

    // ── AHRS attitude ─────────────────────────────────────────────────────────
    bool   use_ahrs_{false};
    bool   ahrs_init_guess_{false};  // use AHRS roll+pitch to correct initial_guess rotation
    bool   ahrs_fuse_yaw_{false};    // also take yaw from AHRS (only for mag-based AHRS)
    bool   has_ahrs_{false};
    std::mutex ahrs_mutex_;
    gtsam::Rot3 latest_ahrs_rot_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr ahrs_sub_;
    double ahrs_noise_rp_{0.02};
    double ahrs_noise_yaw_{0.1};

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
    bool   dvl_static_noise_{true};   // true → use yaml noise_x/y/z; false → use msg covariance
    gtsam::noiseModel::Diagonal::shared_ptr latest_dvl_noise_;  // updated per DVL message
};