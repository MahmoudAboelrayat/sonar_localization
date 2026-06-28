#include "localization_imu_pre.hpp"

GicpOdomNode::GicpOdomNode(const rclcpp::NodeOptions & options) : Node("vgicp_odom_node", options)
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
    ekf_max_age_    = this->declare_parameter<double>("ekf_max_age",    0.1);
    sensor_max_age_ = this->declare_parameter<double>("sensor_max_age", 0.1);

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
    global_map_pub_  = this->create_publisher<sensor_msgs::msg::PointCloud2> (map_pub_topic,        1);
    full_map_pub_    = this->create_publisher<sensor_msgs::msg::PointCloud2> ("vgicp_full_map",     1);
    filtered_pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2> ("filtered_pc", 1);
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
        pc_sub_topic, 1,
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
    int ror_min_neighbors = this->declare_parameter<int>("radius_outlier_removal.min_neighbors_in_radius", 5);
    ror_.setRadiusSearch(ror_radius);
    ror_.setMinNeighborsInRadius(ror_min_neighbors);

    // setup intensity filter (optional)
    filter_intensity = this->declare_parameter<bool>("intensity_filter.filter_intensity", false);
    min_intensity = this->declare_parameter<double>("intensity_filter.min_intensity", 0.0);
    // setup range filter — removes points beyond max_range from sensor origin
    // Useful for stripping sonar far-range artifacts (the flat slab at maximum range).
    filter_range_   = this->declare_parameter<bool>  ("range_filter.filter_range",  false);
    min_range_      = this->declare_parameter<double>("range_filter.min_range",       0.0);
    max_range_      = this->declare_parameter<double>("range_filter.max_range",      10.0);
    // odom noise sigmas [roll, pitch, yaw, x, y, z]
    odom_noise_roll_  = this->declare_parameter<double>("gtsam.odom_noise_roll",  0.1);
    odom_noise_pitch_ = this->declare_parameter<double>("gtsam.odom_noise_pitch", 0.1);
    odom_noise_yaw_   = this->declare_parameter<double>("gtsam.odom_noise_yaw",   0.3);
    odom_noise_x_     = this->declare_parameter<double>("gtsam.odom_noise_x",     0.5);
    odom_noise_y_     = this->declare_parameter<double>("gtsam.odom_noise_y",     0.5);
    odom_noise_z_     = this->declare_parameter<double>("gtsam.odom_noise_z",     0.3);
    vgicp_static_noise_  = this->declare_parameter<bool>  ("gtsam.vgicp_static_noise",  true);
    vgicp_noise_scale_   = this->declare_parameter<double>("gtsam.vgicp_noise_scale",    10.0);
    vgicp_skip_score_    = this->declare_parameter<double>("gtsam.vgicp_skip_score",      0.0);  // 0 = disabled
    vgicp_use_huber_     = this->declare_parameter<bool>  ("gtsam.vgicp_use_huber",      false);
    vgicp_huber_k_       = this->declare_parameter<double>("gtsam.vgicp_huber_k",         1.345);

    lc_noise_roll_    = this->declare_parameter<double>("gtsam.lc_noise_roll",    0.01);
    lc_noise_pitch_   = this->declare_parameter<double>("gtsam.lc_noise_pitch",   0.01);
    lc_noise_yaw_     = this->declare_parameter<double>("gtsam.lc_noise_yaw",     0.01);
    lc_noise_x_       = this->declare_parameter<double>("gtsam.lc_noise_x",       0.05);
    lc_noise_y_       = this->declare_parameter<double>("gtsam.lc_noise_y",       0.05);
    lc_noise_z_       = this->declare_parameter<double>("gtsam.lc_noise_z",       0.05);
    lc_use_huber_     = this->declare_parameter<bool>  ("gtsam.lc_use_huber",     true);
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
    use_dvl_        = this->declare_parameter<bool>("dvl.use_dvl",          false);
    use_dvl_trans_  = this->declare_parameter<bool>("dvl.use_dvl_trans",   false);
    dvl_static_noise_ = this->declare_parameter<bool>("dvl.dvl_static_noise", true);
    dvl_max_vel_    = this->declare_parameter<double>("dvl.max_vel", 3.0);  // reject readings above this [m/s]
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
    use_ahrs_        = this->declare_parameter<bool>  ("ahrs.use_ahrs",         false);
    ahrs_noise_rp_   = this->declare_parameter<double>("ahrs.noise_rp",         0.02);
    ahrs_noise_yaw_  = this->declare_parameter<double>("ahrs.noise_yaw",        0.1);
    ahrs_init_guess_ = this->declare_parameter<bool>  ("ahrs.use_ahrs_init_guess", false);
    ahrs_fuse_yaw_   = this->declare_parameter<bool>  ("ahrs.ahrs_fuse_yaw",    false);
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
        // When AHRS is absent, gravity direction is re-estimated after imu_start_kf_
        // frames via AQUA-SLAM dirG accumulation and then baked into these params.
        imu_preint_params_ = gtsam::PreintegrationParams::MakeSharedD(imu_gravity_);
        imu_preint_params_->accelerometerCovariance = gtsam::I_3x3 * imu_accel_noise_ * imu_accel_noise_;
        imu_preint_params_->gyroscopeCovariance     = gtsam::I_3x3 * imu_gyro_noise_  * imu_gyro_noise_;
        imu_preint_params_->integrationCovariance   = gtsam::I_3x3 * 1e-6;
        preint_      = std::make_shared<gtsam::PreintegratedImuMeasurements>(imu_preint_params_, gtsam::imuBias::ConstantBias());
        scan_preint_ = std::make_shared<gtsam::PreintegratedImuMeasurements>(imu_preint_params_, gtsam::imuBias::ConstantBias());

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

    // Magnetometer yaw — relative heading fused into initial_guess only (no GTSAM factor)
    use_mag_yaw_     = this->declare_parameter<bool>  ("ahrs.use_mag_yaw",       false);
    mag_declination_ = this->declare_parameter<double>("ahrs.mag_declination_deg", 0.0);
    if (use_mag_yaw_) {
        std::string mag_topic = this->declare_parameter<std::string>(
            "topics.mag_sub", "nucleus_node/magnetometer_packets");
        mag_sub_ = this->create_subscription<interfaces::msg::Magnetometer>(
            mag_topic, 100,
            [this](const interfaces::msg::Magnetometer::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(mag_mutex_);
                latest_mag_x_ = msg->magnetometer_x;
                latest_mag_y_ = msg->magnetometer_y;
                latest_mag_z_ = msg->magnetometer_z;
                has_mag_ = true;
            });
        RCLCPP_INFO(get_logger(), "[MagYaw] enabled on %s  declination=%.1f°",
            mag_topic.c_str(), mag_declination_);
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
    // ── Initial guess mode summary ────────────────────────────────────────
    if (use_ekf_) {
        RCLCPP_INFO(get_logger(), "[InitGuess] Mode: EKF odometry");
    } else if (use_imu_) {
        if (use_ahrs_) {
            RCLCPP_INFO(get_logger(),
                "[InitGuess] Mode: IMU preintegration  |  AHRS roll+pitch prior  |  start_kf=%d",
                imu_start_kf_);
        } else {
            RCLCPP_INFO(get_logger(),
                "[InitGuess] Mode: IMU preintegration  |  AQUA-SLAM gravity init  |  start_kf=%d",
                imu_start_kf_);
        }
    } else if (use_dvl_) {
        RCLCPP_INFO(get_logger(), "[InitGuess] Mode: DVL dead-reckoning");
    } else {
        RCLCPP_INFO(get_logger(), "[InitGuess] Mode: scan-to-scan constant velocity");
    }

    if (use_dvl_trans_) RCLCPP_INFO(get_logger(), "[InitGuess] + DVL pre-integrated translation factor");
    if (use_depth_)     RCLCPP_INFO(get_logger(), "[InitGuess] + Depth Z override (%s)",
                            use_ekf_ ? "absolute" : "relative to start");
    if (use_vo_)        RCLCPP_INFO(get_logger(), "[InitGuess] + Visual odometry factor");
    if (use_lc_)        RCLCPP_INFO(get_logger(), "[InitGuess] + Loop closure enabled");

    RCLCPP_INFO(get_logger(), "VGICP SLAM Node initialised.");
}

GicpOdomNode::~GicpOdomNode()
{
    if (loop_closure_thread_.joinable())
        loop_closure_thread_.join();
}
// ── GTSAM initialisation ──────────────────────────────────────────────────
// Always call while holding gtsam_mutex_ (except from constructor)
void GicpOdomNode::initGTSAM()
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
    // Loop closure noise — optionally wrapped in Huber robust kernel
    auto lc_base = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << lc_noise_roll_, lc_noise_pitch_, lc_noise_yaw_,
                             lc_noise_x_,    lc_noise_y_,     lc_noise_z_).finished());
    if (lc_use_huber_)
        robustLoopNoise_ = gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Huber::Create(lc_huber_k_), lc_base);
    else
        robustLoopNoise_ = lc_base;
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
gtsam::Pose3 GicpOdomNode::matrix2Pose3(const Eigen::Matrix4f & m)
{
    return gtsam::Pose3(
        gtsam::Rot3(m.block<3,3>(0,0).cast<double>()),
        gtsam::Point3(m.block<3,1>(0,3).cast<double>()));
}

Eigen::Matrix4f GicpOdomNode::pose32Matrix(const gtsam::Pose3 & p)
{
    Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
    m.block<3,3>(0,0) = p.rotation().matrix().cast<float>();
    m.block<3,1>(0,3) = p.translation().cast<float>();
    return m;
}
// ── Keyframe management ───────────────────────────────────────────────────
void GicpOdomNode::AddKeyFrame(const Eigen::Matrix4f & current_pose,
                 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                 float ekf_z,
                 double stamp_sec,
                 double gicp_score)
{
    // Check keyframe threshold — quick check without GTSAM lock
    {
        std::lock_guard<std::mutex> kf_lock(kf_mutex_);
        if (!keyframes_.empty()) {
            Eigen::Matrix4f delta = keyframes_.back().pose.inverse() * current_pose;
            float dist  = delta.block<3,1>(0,3).norm();
            float _cos = std::max(-1.0f, std::min(1.0f,
                (delta.block<3,3>(0,0).trace() - 1.0f) * 0.5f));
            float angle = std::acos(_cos) * 180.0f / M_PI;
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
        // // ── Bootstrap: capture depth origin so the map frame's Z = 0 ────
        // if (!use_ekf_ && use_depth_) {
        //     std::lock_guard<std::mutex> lk(depth_mutex_);
        //     if (has_depth_) {
        //         depth_origin_ = latest_depth_z_;
        //         RCLCPP_INFO(get_logger(),
        //             "[Depth] origin captured = %.4f m  (all depth readings will be relative to this)",
        //             depth_origin_);
        //     }
        // }
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
                    gtsam::Vector3 dp = preint_->deltaPij();
                    RCLCPP_INFO(get_logger(),
                        "[IMU factor] kf=%d  dt=%.3fs  "
                        "deltaPij=[%.4f, %.4f, %.4f] m  "
                        "deltaVij=[%.4f, %.4f, %.4f] m/s  "
                        "prev_vel=[%.4f, %.4f, %.4f] m/s",
                        current_id, preint_->deltaTij(),
                        dp.x(), dp.y(), dp.z(),
                        dv.x(), dv.y(), dv.z(),
                        prev_velocity_.x(), prev_velocity_.y(), prev_velocity_.z());
                }
                used_imu_factor = true;
            }
        } else if (use_imu_ && current_id < imu_start_kf_) {
            // ── Phase 1: GICP-only warmup, AQUA-SLAM gravity accumulation ──────
            // Accumulate dirG -= R_prev * deltaV_preintegrated across each interval.
            // Real vehicle accelerations cancel over varied motion; what remains
            // after summation is the gravity signal in world frame.
            if (!use_ahrs_ && !gravity_initialized_ && imu_preint_params_) {
                {
                    std::lock_guard<std::mutex> ilk(imu_mutex_);
                    if (preint_ && preint_->deltaTij() > 0.01 && !keyframes_.empty()) {
                        gtsam::Matrix3 R_prev =
                            matrix2Pose3(keyframes_.back().pose).rotation().matrix();
                        dirG_accum_ -= R_prev * preint_->deltaVij();
                        gravity_init_count_++;
                    }
                }

                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                    "[GravInit] kf %d/%d  dirG=[%.3f, %.3f, %.3f]  samples=%d",
                    current_id, imu_start_kf_,
                    dirG_accum_.x(), dirG_accum_.y(), dirG_accum_.z(),
                    gravity_init_count_);
                // ── Phase 2: finalise at last warmup frame ────────────────────
                if (current_id == imu_start_kf_ - 1) {
                    if (gravity_init_count_ >= 3) {
                        gtsam::Vector3 gW = dirG_accum_.normalized();
                        // Bake the estimated gravity direction into the preintegration params.
                        // n_gravity is in the world/nav frame — units: m/s².
                        imu_preint_params_->n_gravity = imu_gravity_ * gW;
                        // Reset both preintegrators so Phase 3 integrates with
                        // the corrected gravity vector from this point forward.
                        {
                            std::lock_guard<std::mutex> ilk(imu_mutex_);
                            preint_      = std::make_shared<gtsam::PreintegratedImuMeasurements>(
                                imu_preint_params_, prev_bias_);
                            scan_preint_ = std::make_shared<gtsam::PreintegratedImuMeasurements>(
                                imu_preint_params_, prev_bias_);
                        }
                        gravity_initialized_ = true;

                        double tilt_deg = std::acos(std::max(-1.0, std::min(1.0,
                            gW.dot(gtsam::Vector3(0, 0, 1))))) * 180.0 / M_PI;
                        RCLCPP_INFO(get_logger(),
                            "[GravInit] Gravity initialised from %d frames. "
                            "gW=[%.4f, %.4f, %.4f]  tilt=%.2f° from Z-down. "
                            "ImuFactor activates next keyframe.",
                            gravity_init_count_,
                            gW.x(), gW.y(), gW.z(), tilt_deg);
                    } else {
                        gravity_initialized_ = true;  // skip retry — keep MakeSharedD default
                        RCLCPP_WARN(get_logger(),
                            "[GravInit] Only %d valid intervals collected — "
                            "keeping default Z-down gravity. "
                            "Increase imu.start_kf or ensure more motion during warmup.",
                            gravity_init_count_);
                    }
                }
            }
            if (debug) {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                    "[IMU warmup] kf=%d/%d — GICP-only, collecting gravity direction",
                    current_id, imu_start_kf_);
            }
        }

        gtsam::Pose3 prev_gtsam = matrix2Pose3(keyframes_.back().pose);
        gtsam::Pose3 relative   = prev_gtsam.between(current_gtsam_pose);
        // Layer 1 — skip BetweenFactor entirely when match is too unreliable.
        // IMU / DVL factors still constrain the pose; we just don't let a bad
        // scan measurement drag the graph.
        bool skip_scan_factor = (vgicp_skip_score_ > 0.0 && gicp_score > vgicp_skip_score_);
        if (skip_scan_factor) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
                "[VGICP factor] kf=%d  SKIPPED (score=%.4f > skip_thresh=%.4f) — "
                "IMU/DVL only this keyframe",
                current_id, gicp_score, vgicp_skip_score_);
        } else {
            // Layer 2 — scale sigma by fitness score so poor-but-accepted matches
            // carry less weight: sigma *= (1 + score * scale).
            gtsam::noiseModel::Diagonal::shared_ptr scan_noise = odomNoise_;
            if (!vgicp_static_noise_ && gicp_score > 0.0) {
                double s = 1.0 + gicp_score * vgicp_noise_scale_;
                scan_noise = gtsam::noiseModel::Diagonal::Sigmas(
                    (gtsam::Vector(6) << odom_noise_roll_  * s, odom_noise_pitch_ * s,
                                         odom_noise_yaw_   * s, odom_noise_x_     * s,
                                         odom_noise_y_     * s, odom_noise_z_     * s).finished());
            }
            // Layer 3 — Huber robust kernel limits the influence of any remaining
            // outlier scans; residuals beyond huber_k are penalised linearly rather
            // than quadratically so a single bad factor cannot dominate the graph.
            gtsam::noiseModel::Base::shared_ptr factor_noise = scan_noise;
            if (vgicp_use_huber_) {
                factor_noise = gtsam::noiseModel::Robust::Create(
                    gtsam::noiseModel::mEstimator::Huber::Create(vgicp_huber_k_),
                    scan_noise);
            }

            gtSAMgraph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
                X(current_id-1), X(current_id), relative, factor_noise));
            if (debug) {
                gtsam::Vector3 dt = relative.translation();
                gtsam::Vector3 rpy = relative.rotation().rpy();
                double scale = (!vgicp_static_noise_ && gicp_score > 0.0)
                    ? (1.0 + gicp_score * vgicp_noise_scale_) : 1.0;
                RCLCPP_INFO(get_logger(),
                    "[VGICP factor] kf=%d  dt=[%.4f, %.4f, %.4f] m  "
                    "drpy=[%.3f, %.3f, %.3f] deg  score=%.4f  noise_scale=%.2f%s",
                    current_id, dt.x(), dt.y(), dt.z(),
                    rpy(0)*180.0/M_PI, rpy(1)*180.0/M_PI, rpy(2)*180.0/M_PI,
                    gicp_score, scale, vgicp_use_huber_ ? "  [Huber]" : "");
            }
        }
        // Also add IMU factor if available — they work together
        if (use_imu_ && preint_ && current_id >= imu_start_kf_) {
            std::lock_guard<std::mutex> ilk(imu_mutex_);
            if (preint_->deltaTij() > 0.01) {
                gtSAMgraph_.add(gtsam::ImuFactor(
                    X(current_id-1), V(current_id-1),
                    X(current_id),   V(current_id),
                    B(current_id-1), *preint_));
                if (debug) {
                    gtsam::Vector3 dv = preint_->deltaVij();
                    gtsam::Vector3 dp = preint_->deltaPij();
                    RCLCPP_INFO(get_logger(),
                        "[IMU factor] kf=%d  dt=%.3fs  "
                        "deltaPij=[%.4f, %.4f, %.4f] m  "
                        "deltaVij=[%.4f, %.4f, %.4f] m/s",
                        current_id, preint_->deltaTij(),
                        dp.x(), dp.y(), dp.z(),
                        dv.x(), dv.y(), dv.z());
                }
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
                auto dvl_factor_noise = (!dvl_static_noise_ && latest_dvl_noise_)
                    ? latest_dvl_noise_ : dvlNoise_;
                gtSAMgraph_.add(DvlVelocityFactor(
                    X(current_id), V(current_id), dvl_snap, dvl_factor_noise));
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
        // Depth factor — constrains Z translation to barometric/pressure depth.
        // When EKF is off, the map frame Z=0 is the starting depth, so subtract
        // depth_origin_ to keep the factor consistent with the initial guess and map frame.
        if (use_depth_) {
            double depth_snap;
            bool depth_ok = false;
            {
                std::lock_guard<std::mutex> lk(depth_mutex_);
                if (has_depth_) { depth_snap = latest_depth_z_; depth_ok = true; }
            }
            if (depth_ok) {
                double depth_z = use_ekf_ ? depth_snap : (depth_snap - depth_origin_);
                gtSAMgraph_.add(DepthFactor(X(current_id), depth_z, depthNoise_));
                if (debug) {
                    RCLCPP_INFO(get_logger(),
                        "[Depth factor] kf=%d  z=%.4f m (raw=%.4f origin=%.4f)",
                        current_id, depth_z, depth_snap, depth_origin_);
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
    try {
        isam_->update(gtSAMgraph_, initialEstimates_);
        isam_->update();
    } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "[GTSAM] ISAM2 update failed at kf=%d: %s — resetting graph",
                     current_id, e.what());
        gtSAMgraph_.resize(0);
        initialEstimates_.clear();
        return;
    }
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
    kf.stamp = rclcpp::Time(static_cast<uint64_t>(stamp_sec * 1e9), RCL_ROS_TIME);
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
void GicpOdomNode::loopClosureThread()
{
    while (rclcpp::ok() && use_lc_) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        performLoopClosure();
    }
}
// ── Loop closure — LIO-SAM style ──────────────────────────────────────────
void GicpOdomNode::performLoopClosure()
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
    if(debug){
        RCLCPP_INFO(get_logger(), "Loop candidate: kf %d -> %d", latest_id, closest_id);
    }
    // ── 2. Downsample history submap ──────────────────────────────────────
    pcl::PointCloud<pcl::PointXYZ>::Ptr history_ds(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> ds_filter;
    ds_filter.setLeafSize(map_res_, map_res_, map_res_);
    ds_filter.setInputCloud(history_cloud_world);
    ds_filter.filter(*history_ds);
    if (history_ds->size() < 20 || latest_cloud_world->size() < 20) {
        RCLCPP_WARN(get_logger(), "Loop closure aborted: too few points (history=%zu latest=%zu).",
                    history_ds->size(), latest_cloud_world->size());
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
        if(debug){
            RCLCPP_WARN(get_logger(), "Loop closure %s did not converge.", lc_matcher);
        }
        return;
    }
    Eigen::Vector3f t_corr     = correction.block<3,1>(0,3);
    float correction_dist      = t_corr.norm();
    float _cos_lc = std::max(-1.0f, std::min(1.0f,
        (correction.block<3,3>(0,0).trace() - 1.0f) * 0.5f));
    float correction_angle = std::acos(_cos_lc) * 180.0f / M_PI;
    if(debug){
        RCLCPP_INFO(get_logger(),
            "LC %s: score=%.4f | correction t=%.2fm angle=%.1fdeg",
            lc_matcher, score, correction_dist, correction_angle);
    }
    if (score > lc_fitness_score_) {
        if(debug){
            RCLCPP_INFO(get_logger(), "Loop closure refused: score %.4f > threshold %.4f",
                        score, lc_fitness_score_);
        }
        return;
    }
    // Reject if correction is unreasonably large — likely wrong minimum
    if (correction_dist > static_cast<float>(lc_max_correction_dist_) ||
        correction_angle > static_cast<float>(lc_max_correction_angle_)) {
            if(debug){
                RCLCPP_WARN(get_logger(),
                    "Loop closure refused: correction too large (t=%.2fm angle=%.1fdeg) "
                    "— likely wrong minimum",
                    correction_dist, correction_angle);
            }
        return;
    }
    if(debug){
        RCLCPP_WARN(get_logger(), "Loop closure accepted! [%s] Score: %.4f | t=%.2fm | angle=%.1fdeg",
                    lc_matcher, score, correction_dist, correction_angle);
    }
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
void GicpOdomNode::updateSubmap()
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
void GicpOdomNode::publishFullMap()
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
void GicpOdomNode::publishPath()
{
    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = odom_frame_;
    path_msg.header.stamp    = this->now();
    for (auto & kf : keyframes_) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header.frame_id = odom_frame_;
        ps.header.stamp    = kf.stamp;

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
void GicpOdomNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
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
    latest_gyro_body_ = gyro;
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
void GicpOdomNode::dvlCallback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
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

    // Update velocity latch (and optionally per-message noise)
    {
        std::lock_guard<std::mutex> lk(dvl_mutex_);
        latest_dvl_vel_body_ = vel;
        has_dvl_ = true;
        if (!dvl_static_noise_) {
            // TwistWithCovariance layout is 6×6 row-major:
            // [vx,vy,vz,wx,wy,wz] — variance_vx at [0], vy at [7], vz at [14]
            const auto& cov = msg->twist.covariance;
            double sx = std::sqrt(std::max(cov[0],  1e-6));
            double sy = std::sqrt(std::max(cov[7],  1e-6));
            double sz = std::sqrt(std::max(cov[14], 1e-6));
            latest_dvl_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
                (gtsam::Vector(3) << sx, sy, sz).finished());
        }
    }

    // ── DVL pre-integration (equation 8, AQUA-SLAM) ──────────────────────
    // Each time a DVL ping arrives, we advance the pre-integrated position
    // using the current accumulated gyro rotation and this velocity sample.
    // The gyro rotation dvl_preint_dR_ is updated in imuCallback below.
    //
    // ΔDi_p̄ += ΔR̂_IiIk * R_ID * Di_v * Δt_dvl
    // We use the DVL ping interval as Δt (typically 0.2 s at 5 Hz).
    // For a more accurate integration, imuCallback accumulates dR between pings.
    rclcpp::Time now = msg->header.stamp;
    double dt_dvl = 0.0;
    if (dvl_last_time_ > 0.0) {
        dt_dvl = now.seconds() - dvl_last_time_;
    }
    dvl_last_time_ = now.seconds();
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
void GicpOdomNode::voCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
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

void GicpOdomNode::ekfCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    Eigen::Quaternionf q(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z);
    if (q.norm() < 1e-6f) return;  // guard against zero-quaternion at EKF startup
    Eigen::Vector3f t(
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z);

    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    pose.block<3,3>(0,0) = q.normalized().toRotationMatrix();
    pose.block<3,1>(0,3) = t;

    std::lock_guard<std::mutex> lock(ekf_mutex_);
    latest_ekf_pose_ = pose;
    latest_ekf_stamp_ = msg->header.stamp;
    has_ekf_ = true;
    ekf_msg_count_++;
}
// ── Point cloud callback ───────────────────────────────────────────────────
void GicpOdomNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
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
    // Sensor sync check — skip scan only if IMU/DVL data is genuinely stale
    // (positive age = sensor stopped publishing). Negative age means the sensor
    // timestamp is ahead of the scan (e.g. sonar timestamps at ping start) — safe to proceed.
    if (use_imu_ || use_dvl_) {
        double scan_t = rclcpp::Time(msg->header.stamp).seconds();
        if (use_imu_ && imu_last_time_ > 0.0) {
            double imu_age = scan_t - imu_last_time_;
            if (imu_age > sensor_max_age_) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "[SyncCheck] IMU is %.3fs stale relative to scan (max %.3fs) — skipping scan",
                    imu_age, sensor_max_age_);
                return;
            }
        }
        if (use_dvl_ && dvl_last_time_ > 0.0) {
            double dvl_age = scan_t - dvl_last_time_;
            if (dvl_age > sensor_max_age_) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "[SyncCheck] DVL is %.3fs stale relative to scan (max %.3fs) — skipping scan",
                    dvl_age, sensor_max_age_);
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
    // 1b. Range filter — drop points farther than max_range from the sensor origin.
    // Applied in sensor frame so the cutoff is always distance from the transducer,
    // independent of how the sensor is tilted.
    pcl::PointCloud<pcl::PointXYZ>::Ptr range_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    if (filter_range_) {
        float min_r2 = static_cast<float>(min_range_ * min_range_);
        float max_r2 = static_cast<float>(max_range_ * max_range_);
        range_filtered->reserve(raw->size());
        for (const auto & pt : *raw) {
            float r2 = pt.x*pt.x + pt.y*pt.y + pt.z*pt.z;
            if (r2 >= min_r2 && r2 <= max_r2)
                range_filtered->push_back(pt);
        }
    } else {
        range_filtered = raw;
    }
    // 2. Transform into base frame
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_base(new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Matrix4f base_transform = is_ned_ ? (ned_transform_ * base2sonar_) : base2sonar_;
    pcl::transformPointCloud(*range_filtered, *cloud_base, base_transform);
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

    if (filtered->size() < 20) return;  // too few points for stable covariance — avoids VGICP SIGFPE
    {
        sensor_msgs::msg::PointCloud2 fpc_msg;
        pcl::toROSMsg(*filtered, fpc_msg);
        fpc_msg.header = msg->header;
        filtered_pc_pub_->publish(fpc_msg);
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
        Eigen::Matrix4f bootstrap_pose = use_ekf_ ? current_ekf_pose : Eigen::Matrix4f::Identity();
        {
            std::lock_guard<std::mutex> pose_lock(pose_mutex_);
            global_pose_ = bootstrap_pose;
        }
        prev_ekf_pose_ = current_ekf_pose;
        // Capture magnetometer heading origin so all subsequent yaw values are relative
        if (!use_ekf_ && use_mag_yaw_ && !mag_yaw_origin_set_) {
            std::lock_guard<std::mutex> lk(mag_mutex_);
            if (has_mag_) {
                mag_yaw_origin_ = std::atan2(
                    -static_cast<double>(latest_mag_y_),
                     static_cast<double>(latest_mag_x_));
                mag_yaw_origin_set_ = true;
                RCLCPP_INFO(get_logger(), "[MagYaw] origin captured = %.2f°",
                    mag_yaw_origin_ * 180.0 / M_PI);
            }
        }

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
    Eigen::Matrix4f initial_guess = current_global;
    if (use_ekf_) {
        // EKF: apply odometry delta from the EKF pose
        Eigen::Matrix4f ekf_delta = prev_ekf_pose_.inverse() * current_ekf_pose;
        initial_guess = current_global * ekf_delta;
    } 
    else if (use_dvl_ && use_imu_ && scan_preint_) {
        // Case 1: DVL + IMU
        //   ΔR = deltaRij() — bias-corrected accumulated rotation at IMU rate (200 Hz)
        //   Δp = scan_dvl_dp_ — DVL displacement integrated in scan-start body frame
        gtsam::Rot3 delta_R;
        {
            std::lock_guard<std::mutex> ilk(imu_mutex_);
            delta_R = scan_preint_->deltaRij();
        }
        Eigen::Matrix3f delta_R_mat = delta_R.matrix().cast<float>();
        Eigen::Vector3f dvl_dp = Eigen::Vector3f::Zero();
        bool dvl_ok = false;
        {
            std::lock_guard<std::mutex> dlk(dvl_mutex_);
            dvl_ok = scan_dvl_valid_;
            dvl_dp = scan_dvl_dp_;
        }
        Eigen::Matrix4f scan_delta = Eigen::Matrix4f::Identity();
        scan_delta.block<3,3>(0,0) = delta_R_mat;
        if (dvl_ok) {
            scan_delta.block<3,1>(0,3) = dvl_dp;
        }
        initial_guess = current_global * scan_delta;
    } 
    else if (use_imu_ && scan_preint_) {
        // Case 2: IMU only (no DVL)
        //   Full preintegration: gravity + double-integrated accel + gyro rotation
        gtsam::NavState prop;
        {
            std::lock_guard<std::mutex> ilk(imu_mutex_);
            prop = scan_preint_->predict(
                gtsam::NavState(matrix2Pose3(current_global), prev_velocity_),
                prev_bias_);
        }
        initial_guess = pose32Matrix(prop.pose());
    } else if (use_dvl_) {
        // DVL only, no IMU — accumulated body-frame displacement, rotation frozen
        Eigen::Vector3f dvl_dp = Eigen::Vector3f::Zero();
        bool dvl_ok = false;
        {
            std::lock_guard<std::mutex> dlk(dvl_mutex_);
            dvl_ok = scan_dvl_valid_;
            dvl_dp = scan_dvl_dp_;
        }
        if (dvl_ok) {
            initial_guess.block<3,1>(0,3) +=
                current_global.block<3,3>(0,0) * dvl_dp;
        }
    } else if (has_prev_scan_) {
        // Case 3: No IMU, no DVL — project last scan-to-scan delta forward
        //   ΔT = T_{k-2}^{-1} * T_{k-1}
        //   T_guess = T_{k-1} * ΔT
        Eigen::Matrix4f delta = prev_scan_pose_.inverse() * current_global;
        initial_guess = current_global * delta;
    }
    // Override Z with depth sensor when EKF is off — more reliable than DVL Z integration.
    // Use relative depth (depth - depth_origin_) so that Z=0 is the starting position,
    // consistent with the map frame. Raw absolute depth would offset every initial guess
    // by the deployment depth, causing VGICP corrections to exceed the rejection threshold.
    if (!use_ekf_ && use_depth_) {
        double depth_snap;
        bool depth_ok = false;
        {
            std::lock_guard<std::mutex> lk(depth_mutex_);
            if (has_depth_) { depth_snap = latest_depth_z_; depth_ok = true; }
        }
        if (depth_ok) initial_guess(2, 3) = static_cast<float>(depth_snap - depth_origin_);
    }
    // ── Magnetometer relative yaw override ───────────────────────────────
    // Only yaw is replaced — roll and pitch from IMU preintegration are untouched.
    // Applied as a world-Z rotation delta so the existing roll/pitch in the rotation
    // matrix are mathematically preserved: R_new = Rz(Δyaw) * R_current.
    // Only active when use_ekf_ is false.
    if (!use_ekf_ && use_mag_yaw_ && has_mag_ && mag_yaw_origin_set_) {
        float mx, my, mz;
        {
            std::lock_guard<std::mutex> lk(mag_mutex_);
            mx = latest_mag_x_; my = latest_mag_y_; mz = latest_mag_z_;
        }
        // Tilt-compensate using roll/pitch from current initial_guess
        gtsam::Rot3 ig_rot = matrix2Pose3(initial_guess).rotation();
        double roll  = ig_rot.roll();
        double pitch = ig_rot.pitch();
        float mx_h = mx * std::cos(pitch) + mz * std::sin(pitch);
        float my_h = mx * std::sin(roll) * std::sin(pitch)
                   + my * std::cos(roll)
                   - mz * std::sin(roll) * std::cos(pitch);

        double mag_yaw_abs = std::atan2(-static_cast<double>(my_h),
                                         static_cast<double>(mx_h))
                           + mag_declination_ * M_PI / 180.0;

        // Relative yaw (zeroed at bootstrap), wrapped to [-π, π]
        double mag_yaw = mag_yaw_abs - mag_yaw_origin_;
        while (mag_yaw >  M_PI) mag_yaw -= 2.0 * M_PI;
        while (mag_yaw < -M_PI) mag_yaw += 2.0 * M_PI;
        // Current yaw in initial_guess
        double cur_yaw = std::atan2(
            static_cast<double>(initial_guess(1, 0)),
            static_cast<double>(initial_guess(0, 0)));
        // Apply only the yaw difference as a pre-multiplied world-Z rotation
        // This preserves roll and pitch exactly.
        double d = mag_yaw - cur_yaw;
        while (d >  M_PI) d -= 2.0 * M_PI;
        while (d < -M_PI) d += 2.0 * M_PI;
        float cd = static_cast<float>(std::cos(d));
        float sd = static_cast<float>(std::sin(d));
        Eigen::Matrix3f Rz;
        Rz <<  cd, -sd, 0.0f,
               sd,  cd, 0.0f,
              0.0f, 0.0f, 1.0f;
        initial_guess.block<3,3>(0,0) = Rz * initial_guess.block<3,3>(0,0);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
            "[MagYaw] abs=%.2f°  rel=%.2f°  cur_yaw=%.2f°  delta=%.2f°",
            mag_yaw_abs * 180.0 / M_PI,
            mag_yaw     * 180.0 / M_PI,
            cur_yaw     * 180.0 / M_PI,
            d           * 180.0 / M_PI);
    }
    // ── Override initial guess rotation with AHRS roll+pitch ─────────────
    // AHRS gives gravity-referenced roll and pitch — no drift, no bias.
    // Keep yaw from IMU preintegration unless ahrs_fuse_yaw_ is set (mag-based AHRS).
    // This prevents accumulated gyro bias from corrupting the initial guess rotation
    // during fast manoeuvres, while still using DVL/IMU for translation.
    // if (!use_ekf_ && ahrs_init_guess_ && has_ahrs_) {
    //     gtsam::Rot3 ahrs_snap;
    //     {
    //         std::lock_guard<std::mutex> lk(ahrs_mutex_);
    //         ahrs_snap = latest_ahrs_rot_;
    //     }
    //     gtsam::Rot3 ig_rot = matrix2Pose3(initial_guess).rotation();
    //     double yaw = ahrs_fuse_yaw_ ? ahrs_snap.yaw() : ig_rot.yaw();
    //     gtsam::Rot3 fused = gtsam::Rot3::RzRyRx(ahrs_snap.roll(), ahrs_snap.pitch(), yaw);
    //     Eigen::Matrix3f fused_mat = fused.matrix().cast<float>();
    //     initial_guess.block<3,3>(0,0) = fused_mat;
    // }
    // ── Compare current initial guess vs what EKF delta would have given ──
    // if (!use_ekf_ && has_ekf_) {
    //     Eigen::Matrix4f ekf_delta = prev_ekf_pose_.inverse() * current_ekf_pose;
    //     Eigen::Matrix4f ekf_guess = current_global * ekf_delta;

    //     Eigen::Vector3f t_cur = initial_guess.block<3,1>(0,3);
    //     Eigen::Vector3f t_ekf = ekf_guess.block<3,1>(0,3);
    //     Eigen::Vector3f diff  = t_cur - t_ekf;

    //     // Extract yaw from each rotation matrix (Z-Y-X Euler, yaw = atan2(R10, R00))
    //     float yaw_cur = std::atan2(initial_guess(1,0), initial_guess(0,0)) * 180.f / M_PI;
    //     float yaw_ekf = std::atan2(ekf_guess(1,0),    ekf_guess(0,0))     * 180.f / M_PI;
    //     float yaw_diff = yaw_cur - yaw_ekf;
    //     // Wrap to [-180, 180]
    //     if (yaw_diff >  180.f) yaw_diff -= 360.f;
    //     if (yaw_diff < -180.f) yaw_diff += 360.f;

    //     RCLCPP_INFO(get_logger(),
    //         "[InitGuess cmp]  xyz_cur=[%.3f, %.3f, %.3f]  xyz_ekf=[%.3f, %.3f, %.3f]"
    //         "  xyz_diff=[%.3f, %.3f, %.3f] m  |diff|=%.3f m"
    //         "  yaw_cur=%.2f°  yaw_ekf=%.2f°  yaw_diff=%.2f°",
    //         t_cur.x(), t_cur.y(), t_cur.z(),
    //         t_ekf.x(), t_ekf.y(), t_ekf.z(),
    //         diff.x(),  diff.y(),  diff.z(),  diff.norm(),
    //         yaw_cur, yaw_ekf, yaw_diff);
    // }
    // 7. Snapshot local map
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_snapshot;
    {
        std::lock_guard<std::mutex> kf_lock(kf_mutex_);
        map_snapshot = local_map_;
    }
    // 8. Run GICP
    // Eigen::Vector3f    t_ig(initial_guess.block<3,1>(0,3));
    // Eigen::Quaternionf q_ig(initial_guess.block<3,3>(0,0));
    // Eigen::Vector3f    rpy = q_ig.toRotationMatrix().eulerAngles(0, 1, 2)
    //                             * (180.0f / M_PI);
    // RCLCPP_INFO(get_logger(),
    //     "[InitGuess] xyz=[%.3f, %.3f, %.3f]  rpy=[%.2f, %.2f, %.2f] deg",
    //     t_ig.x(), t_ig.y(), t_ig.z(),
    //     rpy.x(), rpy.y(), rpy.z());

    if (!map_snapshot || map_snapshot->size() < 20) return;
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
    float _cos_vc = std::max(-1.0f, std::min(1.0f,
        (diff.block<3,3>(0,0).trace() - 1.0f) * 0.5f));
    float correction_angle        = std::acos(_cos_vc) * 180.0f / M_PI;

    bool gicp_rejected = converged && (
        (gicp_fitness_score_ > 0.0 && gicp_score > gicp_fitness_score_) ||
        correction_dist  > static_cast<float>(gicp_max_correction_dist_) ||
        correction_angle > static_cast<float>(gicp_max_correction_angle_));
    // if (debug && converged) {
    //     RCLCPP_INFO(get_logger(),
    //         "GICP | score: %.4f | correction: t=%.2fm angle=%.1fdeg | src: %zu | tgt: %zu",
    //         gicp_score, correction_dist, correction_angle,
    //         filtered->size(), map_snapshot->size());
    // }
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
                    rclcpp::Time(msg->header.stamp).seconds(), score);
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
        prev_ekf_pose_ = current_ekf_pose;   // always update — used for comparison print even when !use_ekf_
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
                if(use_ekf_){
                // On rejection, keep the IMU-predicted rotation from initial_guess.
                // initial_guess is computed before VGICP runs, so it carries no VGICP
                // corruption. Freezing to current_global.rotation would silently drop every
                // rejected frame's rotation, causing multi-frame error accumulation during
                // consecutive rejections (e.g. fast rotation → 5°/frame × 3 rejects = 15°).
                Eigen::Matrix4f safe_guess = initial_guess;
                safe_guess.block<3,3>(0,0) = current_global.block<3,3>(0,0);
                global_pose_ = safe_guess;
                }else{
                // On rejection, keep the IMU-predicted rotation from initial_guess.
                // initial_guess is computed before VGICP runs, so it carries no VGICP
                // corruption. Freezing to current_global.rotation would silently drop every
                // rejected frame's rotation, causing multi-frame error accumulation during
                // consecutive rejections (e.g. fast rotation → 5°/frame × 3 rejects = 15°).
                Eigen::Matrix4f safe_guess = initial_guess;
                global_pose_ = safe_guess;}
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
            prev_ekf_pose_ = current_ekf_pose;
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
void GicpOdomNode::publishOdometry(const std_msgs::msg::Header & header, double fitness_score, bool is_global_match)
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

void GicpOdomNode::publishLoopConstraints(int latest_id, int closest_id)
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