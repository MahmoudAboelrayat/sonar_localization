#include <memory>
#include <string>
#include <mutex>
#include <vector>
#include <chrono>
#include <numeric>
#include <algorithm>
#include <iomanip>
#include <sstream>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/common/transforms.h>

#include <fast_gicp/gicp/fast_vgicp.hpp>
#include <fast_gicp/gicp/fast_gicp.hpp>
#include <Eigen/Geometry>

// ─────────────────────────────────────────────────────────────────────────────
//  Registration algorithm enum
// ─────────────────────────────────────────────────────────────────────────────
enum class RegMethod {
    ICP,           // Point-to-point ICP
    POINT_TO_PLANE,// Point-to-plane ICP
    GICP,          // Generalised ICP (fast_gicp)
    VGICP,         // Voxelised GICP (fast_gicp) — original algorithm
    NDT            // Normal Distributions Transform
};

std::string methodName(RegMethod m)
{
    switch (m) {
        case RegMethod::ICP:            return "ICP";
        case RegMethod::POINT_TO_PLANE: return "PointToPlane";
        case RegMethod::GICP:           return "GICP";
        case RegMethod::VGICP:          return "VGICP";
        case RegMethod::NDT:            return "NDT";
        default:                        return "Unknown";
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Per-algorithm statistics
// ─────────────────────────────────────────────────────────────────────────────
struct AlgorithmStats {
    std::string name;
    int   total_scans{0};
    int   failed_scans{0};
    int   converged_scans{0};
    std::vector<double> times_ms;        // per-scan registration time [ms]
    std::vector<double> scores;          // fitness score per converged scan

    void recordSuccess(double time_ms, double score)
    {
        total_scans++;
        converged_scans++;
        times_ms.push_back(time_ms);
        scores.push_back(score);
    }

    void recordFailure(double time_ms)
    {
        total_scans++;
        failed_scans++;
        times_ms.push_back(time_ms);
    }

    // ── Summary string ────────────────────────────────────────────────────────
    std::string summary() const
    {
        if (times_ms.empty()) return name + ": no data";

        double mean_t = std::accumulate(times_ms.begin(), times_ms.end(), 0.0)
                        / times_ms.size();

        std::vector<double> sorted_t = times_ms;
        std::sort(sorted_t.begin(), sorted_t.end());
        double median_t = sorted_t[sorted_t.size() / 2];
        double max_t    = sorted_t.back();
        double min_t    = sorted_t.front();

        double mean_score = 0.0;
        if (!scores.empty())
            mean_score = std::accumulate(scores.begin(), scores.end(), 0.0)
                         / scores.size();

        double success_rate = total_scans > 0
            ? 100.0 * converged_scans / total_scans : 0.0;

        std::ostringstream ss;
        ss << std::fixed << std::setprecision(2);
        ss << "\n══════════════════════════════════════════════════\n";
        ss << "  Algorithm:       " << name             << "\n";
        ss << "──────────────────────────────────────────────────\n";
        ss << "  Total scans:     " << total_scans      << "\n";
        ss << "  Converged:       " << converged_scans
           << "  (" << success_rate << "%)\n";
        ss << "  Failed:          " << failed_scans     << "\n";
        ss << "──────────────────────────────────────────────────\n";
        ss << "  Time mean:       " << mean_t    << " ms\n";
        ss << "  Time median:     " << median_t  << " ms\n";
        ss << "  Time min:        " << min_t     << " ms\n";
        ss << "  Time max:        " << max_t     << " ms\n";
        ss << "  Mean score:      " << mean_score        << "\n";
        ss << "══════════════════════════════════════════════════\n";
        return ss.str();
    }
};

// ─────────────────────────────────────────────────────────────────────────────
//  Keyframe
// ─────────────────────────────────────────────────────────────────────────────
struct Keyframe {
    Eigen::Matrix4f pose;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
};

// ─────────────────────────────────────────────────────────────────────────────
//  Node
// ─────────────────────────────────────────────────────────────────────────────
class GicpOdomNode : public rclcpp::Node
{
public:
    GicpOdomNode() : Node("vgicp_odom_node")
    {
        // ── Topics ────────────────────────────────────────────────────────────
        std::string odom_pub_topic      = declare_parameter<std::string>("topics.odom_pub",          "vgicp_odom");
        std::string pc_sub_topic        = declare_parameter<std::string>("topics.pc_sub",            "/sonar/point_cloud");
        std::string deadreckoning_sub   = declare_parameter<std::string>("topics.deadreckoining_sub","/odometry/filtered");
        std::string map_pub_topic       = declare_parameter<std::string>("topics.map_pub",           "vgicp_global_map");
        std::string history_map_topic   = declare_parameter<std::string>("topics.history_map_pub",   "vgicp_full_map");
        std::string filtered_pc_topic   = declare_parameter<std::string>("topics.filtered_pc_pub",   "filtered_cloud");

        // ── Registration algorithm selection ──────────────────────────────────
        // Options: "ICP", "PointToPlane", "GICP", "VGICP", "NDT"
        std::string method_str = declare_parameter<std::string>("registration.method", "VGICP");
        method_ = parseMethod(method_str);
        stats_.name = methodName(method_);
        RCLCPP_INFO(get_logger(), "Registration method: %s", stats_.name.c_str());

        // ── Tuning ────────────────────────────────────────────────────────────
        double map_res         = declare_parameter<double>("tuning.map_res",          0.1);
        int    threads         = declare_parameter<int>   ("tuning.vgicp_threads",    4);
        double epsilon         = declare_parameter<double>("tuning.vgicp_epsilon",    1e-4);
        double max_dist        = declare_parameter<double>("tuning.vgicp_max_dist",   1.5);
        int    max_iter        = declare_parameter<int>   ("tuning.vgicp_max_iter",   50);
        double vgicp_res       = declare_parameter<double>("tuning.vgicp_resolution", 0.25);
        max_lost_frames_       = declare_parameter<int>   ("tuning.max_lost_frames",  40);

        // NDT-specific
        double ndt_res         = declare_parameter<double>("tuning.ndt_resolution",  1.0);
        double ndt_step        = declare_parameter<double>("tuning.ndt_step_size",   0.5);
        double ndt_eps         = declare_parameter<double>("tuning.ndt_epsilon",     0.01);
        int    ndt_iter        = declare_parameter<int>   ("tuning.ndt_max_iter",    50);

        // Point-to-plane normal estimation
        normal_k_search_       = declare_parameter<int>   ("tuning.normal_k_search", 20);

        // ── Frames ────────────────────────────────────────────────────────────
        odom_frame_ = declare_parameter<std::string>("frames.odom_frame", "odom");
        base_frame_ = declare_parameter<std::string>("frames.base_frame", "base_link");
        is_ned_        = declare_parameter<bool>("is_ned",         false);
        ekf_z_         = declare_parameter<bool>("ekf_z",          false);
        ekf_fallback_  = declare_parameter<bool>("ekf_fallback",   true);

        // ── Keyframe ──────────────────────────────────────────────────────────
        submap_size_     = declare_parameter<int>   ("keyframe.submap_size",  20);
        kf_dist_thresh_  = declare_parameter<double>("keyframe.dist_thresh",  0.5);
        kf_angle_thresh_ = declare_parameter<double>("keyframe.angle_thresh", 10.0);

        // ── Outlier removal ───────────────────────────────────────────────────
        filter_outliers_        = declare_parameter<bool>  ("outlier_removal.filter_outliers",               true);
        int    num_neighbors    = declare_parameter<int>   ("outlier_removal.num_neighbors",                 20);
        double stddev_thresh    = declare_parameter<double>("outlier_removal.stddev_mul_thresh",             1.5);
        filter_radius_outliers_ = declare_parameter<bool>  ("radius_outlier_removal.filter_radius_outliers", true);
        double ror_radius       = declare_parameter<double>("radius_outlier_removal.search_radius",          0.5);
        int    ror_min_n        = declare_parameter<int>   ("radius_outlier_removal.min_neighbors",          5);
        filter_intensity_       = declare_parameter<bool>  ("intensity_filter.filter_intensity",             false);
        min_intensity_          = declare_parameter<double>("intensity_filter.min_intensity",                0.0);

        sor_.setMeanK(num_neighbors);
        sor_.setStddevMulThresh(stddev_thresh);
        ror_.setRadiusSearch(ror_radius);
        ror_.setMinNeighborsInRadius(ror_min_n);

        // ── Extrinsics ────────────────────────────────────────────────────────
        double s2b_x     = declare_parameter<double>("tf.sonar2base_x",     -0.545);
        double s2b_y     = declare_parameter<double>("tf.sonar2base_y",      0.000);
        double s2b_z     = declare_parameter<double>("tf.sonar2base_z",     -0.404);
        double s2b_roll  = declare_parameter<double>("tf.sonar2base_roll",   0.0);
        double s2b_pitch = declare_parameter<double>("tf.sonar2base_pitch", -30.0);
        double s2b_yaw   = declare_parameter<double>("tf.sonar2base_yaw",    0.0);

        Eigen::Quaternionf q_sb =
            Eigen::AngleAxisf(static_cast<float>(s2b_yaw   * M_PI/180.0), Eigen::Vector3f::UnitZ())
          * Eigen::AngleAxisf(static_cast<float>(s2b_pitch * M_PI/180.0), Eigen::Vector3f::UnitY())
          * Eigen::AngleAxisf(static_cast<float>(s2b_roll  * M_PI/180.0), Eigen::Vector3f::UnitX());

        sonar2base_.setIdentity();
        sonar2base_.block<3,3>(0,0) = q_sb.toRotationMatrix();
        sonar2base_.block<3,1>(0,3) = Eigen::Vector3f(s2b_x, s2b_y, s2b_z);
        base2sonar_ = sonar2base_.inverse();

        ned_transform_ << 1,  0,  0,  0,
                          0, -1,  0,  0,
                          0,  0, -1,  0,
                          0,  0,  0,  1;

        // ── Configure registration algorithms ─────────────────────────────────

        // VGICP (fast_gicp)
        vgicp_.setNumThreads(threads);
        vgicp_.setTransformationEpsilon(epsilon);
        vgicp_.setMaxCorrespondenceDistance(max_dist);
        vgicp_.setMaximumIterations(max_iter);
        vgicp_.setResolution(vgicp_res);

        // GICP (fast_gicp)
        gicp_.setNumThreads(threads);
        gicp_.setTransformationEpsilon(epsilon);
        gicp_.setMaxCorrespondenceDistance(max_dist);
        gicp_.setMaximumIterations(max_iter);

        // Point-to-point ICP (PCL)
        icp_.setTransformationEpsilon(epsilon);
        icp_.setMaxCorrespondenceDistance(max_dist);
        icp_.setMaximumIterations(max_iter);
        icp_.setEuclideanFitnessEpsilon(1e-6);

        // Point-to-plane ICP (PCL NonLinear ICP with normals)
        icp_pl_.setTransformationEpsilon(epsilon);
        icp_pl_.setMaxCorrespondenceDistance(max_dist);
        icp_pl_.setMaximumIterations(max_iter);
        icp_pl_.setEuclideanFitnessEpsilon(1e-6);

        // NDT (PCL)
        ndt_.setResolution(ndt_res);
        ndt_.setStepSize(ndt_step);
        ndt_.setTransformationEpsilon(ndt_eps);
        ndt_.setMaximumIterations(ndt_iter);

        // ── State ─────────────────────────────────────────────────────────────
        global_pose_     = Eigen::Matrix4f::Identity();
        latest_ekf_pose_ = Eigen::Matrix4f::Identity();
        prev_ekf_pose_   = Eigen::Matrix4f::Identity();
        local_map_       = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        map_filter_.setLeafSize(map_res, map_res, map_res);

        // ── Publishers ────────────────────────────────────────────────────────
        odom_pub_         = create_publisher<nav_msgs::msg::Odometry>       (odom_pub_topic,    10);
        global_map_pub_   = create_publisher<sensor_msgs::msg::PointCloud2> (map_pub_topic,      1);
        history_map_pub_  = create_publisher<sensor_msgs::msg::PointCloud2> (history_map_topic,  1);
        filtered_pc_pub_  = create_publisher<sensor_msgs::msg::PointCloud2> (filtered_pc_topic, 10);
        stats_pub_        = create_publisher<std_msgs::msg::String>         ("registration_stats", 10);

        // ── Subscribers ───────────────────────────────────────────────────────
        pc_sub_  = create_subscription<sensor_msgs::msg::PointCloud2>(
            pc_sub_topic, 10,
            std::bind(&GicpOdomNode::pointCloudCallback, this, std::placeholders::_1));
        ekf_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            deadreckoning_sub, 10,
            std::bind(&GicpOdomNode::ekfCallback, this, std::placeholders::_1));

        // ── Stats timer: print summary every 30 seconds ───────────────────────
        stats_timer_ = create_wall_timer(
            std::chrono::seconds(30),
            std::bind(&GicpOdomNode::publishStats, this));

        RCLCPP_INFO(get_logger(),
            "Registration benchmark node started. Method: [%s]", stats_.name.c_str());
    }

    ~GicpOdomNode()
    {
        // Print final statistics on shutdown
        RCLCPP_INFO(get_logger(), "%s", stats_.summary().c_str());
    }

private:

    // ── Algorithm selection ───────────────────────────────────────────────────
    RegMethod parseMethod(const std::string& s)
    {
        std::string u = s;
        std::transform(u.begin(), u.end(), u.begin(), ::toupper);
        if (u == "ICP")            return RegMethod::ICP;
        if (u == "POINTTOPLANE")   return RegMethod::POINT_TO_PLANE;
        if (u == "GICP")           return RegMethod::GICP;
        if (u == "NDT")            return RegMethod::NDT;
        return RegMethod::VGICP;
    }

    // ── Normal estimation for point-to-plane ────────git ─────────────────────────
    // Converts PointXYZI cloud to PointNormal cloud for ICP point-to-plane
    pcl::PointCloud<pcl::PointNormal>::Ptr estimateNormals(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
    {
        // Convert XYZI → XYZ for normal estimation
        pcl::PointCloud<pcl::PointXYZ>::Ptr xyz(new pcl::PointCloud<pcl::PointXYZ>);
        xyz->reserve(cloud->size());
        for (const auto& pt : *cloud) {
            pcl::PointXYZ p;
            p.x = pt.x; p.y = pt.y; p.z = pt.z;
            xyz->push_back(p);
        }

        // Estimate normals
        pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
        ne.setKSearch(normal_k_search_);
        ne.setInputCloud(xyz);
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        ne.compute(*normals);

        // Concatenate XYZ + normals → PointNormal
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(
            new pcl::PointCloud<pcl::PointNormal>);
        cloud_with_normals->resize(cloud->size());
        for (size_t i = 0; i < cloud->size(); ++i) {
            cloud_with_normals->points[i].x         = cloud->points[i].x;
            cloud_with_normals->points[i].y         = cloud->points[i].y;
            cloud_with_normals->points[i].z         = cloud->points[i].z;
            cloud_with_normals->points[i].normal_x  = normals->points[i].normal_x;
            cloud_with_normals->points[i].normal_y  = normals->points[i].normal_y;
            cloud_with_normals->points[i].normal_z  = normals->points[i].normal_z;
        }
        return cloud_with_normals;
    }

    // ── Convert PointXYZI → PointXYZ (for algorithms that don't need intensity)
    pcl::PointCloud<pcl::PointXYZ>::Ptr toXYZ(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>);
        out->reserve(cloud->size());
        for (const auto& pt : *cloud) {
            out->push_back(pcl::PointXYZ(pt.x, pt.y, pt.z));
        }
        return out;
    }

    // ── Run registration and return (converged, score, final_transform) ───────
    struct RegResult {
        bool            converged{false};
        double          score{1e9};
        Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    };

    RegResult runRegistration(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& source,
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& target,
        const Eigen::Matrix4f&                       initial_guess)
    {
        RegResult res;

        switch (method_) {

        // ── ICP: point-to-point ───────────────────────────────────────────────
        case RegMethod::ICP: {
            auto src_xyz = toXYZ(source);
            auto tgt_xyz = toXYZ(target);
            icp_.setInputSource(src_xyz);
            icp_.setInputTarget(tgt_xyz);
            pcl::PointCloud<pcl::PointXYZ> aligned;
            icp_.align(aligned, initial_guess);
            res.converged = icp_.hasConverged();
            res.score     = icp_.getFitnessScore();
            res.T         = icp_.getFinalTransformation();
            break;
        }

        // ── Point-to-plane ICP ────────────────────────────────────────────────
        case RegMethod::POINT_TO_PLANE: {
            // Estimate normals on target — normals define the plane constraint
            auto src_n = estimateNormals(source);
            auto tgt_n = estimateNormals(target);
            icp_pl_.setInputSource(src_n);
            icp_pl_.setInputTarget(tgt_n);
            pcl::PointCloud<pcl::PointNormal> aligned;
            icp_pl_.align(aligned, initial_guess);
            res.converged = icp_pl_.hasConverged();
            res.score     = icp_pl_.getFitnessScore();
            res.T         = icp_pl_.getFinalTransformation();
            break;
        }

        // ── GICP ──────────────────────────────────────────────────────────────
        case RegMethod::GICP: {
            auto src_xyz = toXYZ(source);
            auto tgt_xyz = toXYZ(target);
            gicp_.setInputSource(src_xyz);
            gicp_.setInputTarget(tgt_xyz);
            pcl::PointCloud<pcl::PointXYZ> aligned;
            gicp_.align(aligned, initial_guess);
            res.converged = gicp_.hasConverged();
            res.score     = gicp_.getFitnessScore();
            res.T         = gicp_.getFinalTransformation();
            break;
        }

        // ── VGICP (default) ───────────────────────────────────────────────────
        case RegMethod::VGICP: {
            vgicp_.setInputSource(source);
            vgicp_.setInputTarget(target);
            pcl::PointCloud<pcl::PointXYZI> aligned;
            vgicp_.align(aligned, initial_guess);
            res.converged = vgicp_.hasConverged();
            res.score     = vgicp_.getFitnessScore();
            res.T         = vgicp_.getFinalTransformation();
            break;
        }

        // ── NDT ───────────────────────────────────────────────────────────────
        case RegMethod::NDT: {
            auto src_xyz = toXYZ(source);
            auto tgt_xyz = toXYZ(target);
            ndt_.setInputSource(src_xyz);
            ndt_.setInputTarget(tgt_xyz);
            pcl::PointCloud<pcl::PointXYZ> aligned;
            ndt_.align(aligned, initial_guess);
            res.converged = ndt_.hasConverged();
            res.score     = ndt_.getFitnessScore();
            res.T         = ndt_.getFinalTransformation();
            break;
        }
        }

        return res;
    }

    // ── Submap ────────────────────────────────────────────────────────────────
    void updateSubmap()
    {
        local_map_->clear();
        int start = std::max(0, static_cast<int>(keyframes_.size()) - submap_size_);
        for (int i = start; i < static_cast<int>(keyframes_.size()); ++i) {
            pcl::PointCloud<pcl::PointXYZI> transformed;
            pcl::transformPointCloud(*keyframes_[i].cloud, transformed, keyframes_[i].pose);
            *local_map_ += transformed;
        }
        pcl::PointCloud<pcl::PointXYZI>::Ptr ds(new pcl::PointCloud<pcl::PointXYZI>);
        map_filter_.setInputCloud(local_map_);
        map_filter_.filter(*ds);
        local_map_ = ds;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr buildFullMap()
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr full(new pcl::PointCloud<pcl::PointXYZI>);
        for (const auto& kf : keyframes_) {
            pcl::PointCloud<pcl::PointXYZI> transformed;
            pcl::transformPointCloud(*kf.cloud, transformed, kf.pose);
            *full += transformed;
        }
        pcl::PointCloud<pcl::PointXYZI>::Ptr ds(new pcl::PointCloud<pcl::PointXYZI>);
        map_filter_.setInputCloud(full);
        map_filter_.filter(*ds);
        return ds;
    }

    void addKeyFrame(const Eigen::Matrix4f& pose,
                      const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
    {
        if (keyframes_.empty()) {
            Keyframe kf; kf.pose = pose; kf.cloud = cloud;
            keyframes_.push_back(kf);
            updateSubmap();
            return;
        }
        Eigen::Matrix4f delta = keyframes_.back().pose.inverse() * pose;
        float dist  = delta.block<3,1>(0,3).norm();
        float angle = Eigen::AngleAxisf(
            Eigen::Matrix3f(delta.block<3,3>(0,0))).angle() * 180.0f / M_PI;
        if (dist > static_cast<float>(kf_dist_thresh_) ||
            angle > static_cast<float>(kf_angle_thresh_)) {
            Keyframe kf; kf.pose = pose; kf.cloud = cloud;
            keyframes_.push_back(kf);
            updateSubmap();
            RCLCPP_DEBUG(get_logger(),
                "Keyframe %zu added. Submap: %zu pts",
                keyframes_.size(), local_map_->size());
        }
    }

    // ── Statistics publisher ──────────────────────────────────────────────────
    void publishStats()
    {
        std::string s = stats_.summary();
        RCLCPP_INFO(get_logger(), "%s", s.c_str());
        std_msgs::msg::String msg;
        msg.data = s;
        stats_pub_->publish(msg);
    }

    // ── EKF callback ──────────────────────────────────────────────────────────
    void ekfCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(ekf_mutex_);
        Eigen::Quaternionf q(
            msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
        Eigen::Vector3f t(
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z);
        latest_ekf_pose_.setIdentity();
        latest_ekf_pose_.block<3,3>(0,0) = q.toRotationMatrix();
        latest_ekf_pose_.block<3,1>(0,3) = t;
        has_ekf_ = true;
    }

    // ── Point cloud callback ──────────────────────────────────────────────────
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        auto t_start = std::chrono::high_resolution_clock::now();

        // ── Detect intensity field once ───────────────────────────────────────
        if (has_intensity_ == -1) {
            has_intensity_ = 0;
            for (const auto& f : msg->fields)
                if (f.name == "intensity") { has_intensity_ = 1; break; }
        }

        // ── Convert ──────────────────────────────────────────────────────────
        pcl::PointCloud<pcl::PointXYZI>::Ptr raw(new pcl::PointCloud<pcl::PointXYZI>);
        if (has_intensity_) {
            pcl::fromROSMsg(*msg, *raw);
        } else {
            pcl::PointCloud<pcl::PointXYZ> xyz;
            pcl::fromROSMsg(*msg, xyz);
            for (const auto& pt : xyz) {
                pcl::PointXYZI p;
                p.x = pt.x; p.y = pt.y; p.z = pt.z; p.intensity = 1.0f;
                raw->push_back(p);
            }
        }

        // ── Intensity filter ──────────────────────────────────────────────────
        pcl::PointCloud<pcl::PointXYZI>::Ptr intensity_filtered(
            new pcl::PointCloud<pcl::PointXYZI>);
        if (filter_intensity_) {
            for (const auto& pt : *raw)
                if (std::isfinite(pt.x) && pt.intensity > min_intensity_)
                    intensity_filtered->push_back(pt);
        } else {
            intensity_filtered = raw;
        }
        if (intensity_filtered->empty()) return;

        // ── Transform sonar → base_link ───────────────────────────────────────
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_base(
            new pcl::PointCloud<pcl::PointXYZI>);
        Eigen::Matrix4f T_base = is_ned_
            ? (ned_transform_ * base2sonar_) : base2sonar_;
        pcl::transformPointCloud(*intensity_filtered, *cloud_base, T_base);

        // ── Radius outlier removal ────────────────────────────────────────────
        pcl::PointCloud<pcl::PointXYZI>::Ptr ror_out(
            new pcl::PointCloud<pcl::PointXYZI>);
        if (filter_radius_outliers_) {
            ror_.setInputCloud(cloud_base);
            ror_.filter(*ror_out);
        } else { ror_out = cloud_base; }

        // ── Statistical outlier removal ───────────────────────────────────────
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(
            new pcl::PointCloud<pcl::PointXYZI>);
        if (filter_outliers_ && ror_out->size() > 10) {
            sor_.setInputCloud(ror_out);
            sor_.filter(*filtered);
        } else { filtered = ror_out; }

        if (filtered->empty()) return;

        // ── EKF snapshot ──────────────────────────────────────────────────────
        Eigen::Matrix4f ekf_pose;
        { std::lock_guard<std::mutex> lk(ekf_mutex_); ekf_pose = latest_ekf_pose_; }

        // ── Bootstrap ─────────────────────────────────────────────────────────
        if (local_map_->empty()) {
            global_pose_   = ekf_pose;
            prev_ekf_pose_ = ekf_pose;
            addKeyFrame(global_pose_, filtered);
            RCLCPP_INFO(get_logger(), "Map initialised. Method: [%s]",
                        stats_.name.c_str());
            return;
        }

        // ── Initial guess from EKF delta ──────────────────────────────────────
        Eigen::Matrix4f ekf_delta     = prev_ekf_pose_.inverse() * ekf_pose;
        Eigen::Matrix4f initial_guess = global_pose_ * ekf_delta;

        // ── Run selected registration algorithm ───────────────────────────────
        RegResult res = runRegistration(filtered, local_map_, initial_guess);

        // ── Measure time ──────────────────────────────────────────────────────
        auto t_end   = std::chrono::high_resolution_clock::now();
        double time_ms = std::chrono::duration<double, std::milli>(
            t_end - t_start).count();

        if (res.converged) {
            // ── Record success ────────────────────────────────────────────────
            stats_.recordSuccess(time_ms, res.score);
            lost_frames_ = 0;

            global_pose_ = res.T;
            if (ekf_z_) global_pose_(2,3) = ekf_pose(2,3);

            publishOdometry(msg->header);
            addKeyFrame(global_pose_, filtered);
            prev_ekf_pose_ = ekf_pose;

            RCLCPP_INFO(get_logger(),
                "[%s] Converged | time: %.1fms | score: %.4f | "
                "total: %d | failed: %d | avg: %.1fms",
                stats_.name.c_str(), time_ms, res.score,
                stats_.total_scans, stats_.failed_scans,
                stats_.times_ms.empty() ? 0.0
                    : std::accumulate(stats_.times_ms.begin(),
                                      stats_.times_ms.end(), 0.0)
                      / stats_.times_ms.size());

            // Publish submap every 5 scans
            static int map_count = 0;
            if (map_count++ % 5 == 0) {
                sensor_msgs::msg::PointCloud2 map_msg;
                pcl::toROSMsg(*local_map_, map_msg);
                map_msg.header.frame_id = odom_frame_;
                map_msg.header.stamp    = msg->header.stamp;
                global_map_pub_->publish(map_msg);
            }

            // Publish full map every 20 scans
            static int hist_count = 0;
            if (hist_count++ % 20 == 0) {
                auto full = buildFullMap();
                sensor_msgs::msg::PointCloud2 hist_msg;
                pcl::toROSMsg(*full, hist_msg);
                hist_msg.header.frame_id = odom_frame_;
                hist_msg.header.stamp    = msg->header.stamp;
                history_map_pub_->publish(hist_msg);
            }

        } else {
            // ── Record failure ────────────────────────────────────────────────
            stats_.recordFailure(time_ms);

            RCLCPP_WARN(get_logger(),
                "[%s] FAILED to converge | time: %.1fms | "
                "total failed: %d / %d scans",
                stats_.name.c_str(), time_ms,
                stats_.failed_scans, stats_.total_scans);

            if (lost_frames_ < max_lost_frames_) {
                ++lost_frames_;
                if (ekf_fallback_) {
                    global_pose_   = initial_guess;
                    prev_ekf_pose_ = ekf_pose;
                    publishOdometry(msg->header);
                }
                // else: keep global_pose_ at last good registration;
                //       prev_ekf_pose_ unchanged so delta accumulates for next frame
            } else {
                RCLCPP_ERROR(get_logger(),
                    "Tracking lost — clearing map and resetting.");
                lost_frames_ = 0;
                local_map_->clear();
                keyframes_.clear();
                reset_counter_++;
            }
        }
    }

    // ── Odometry publisher ────────────────────────────────────────────────────
    void publishOdometry(const std_msgs::msg::Header& header)
    {
        Eigen::Vector3f    t(global_pose_.block<3,1>(0,3));
        Eigen::Quaternionf q(global_pose_.block<3,3>(0,0));
        q.normalize();

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
        odom_pub_->publish(odom);
    }

    // ── Members ───────────────────────────────────────────────────────────────

    // ROS
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr  pc_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr         ekf_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr            odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr      global_map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr      history_map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr      filtered_pc_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr              stats_pub_;
    rclcpp::TimerBase::SharedPtr                                     stats_timer_;

    // Registration algorithms
    RegMethod method_{RegMethod::VGICP};
    fast_gicp::FastVGICP<pcl::PointXYZI, pcl::PointXYZI>           vgicp_;
    fast_gicp::FastGICP<pcl::PointXYZ,   pcl::PointXYZ>            gicp_;
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>       icp_;
    pcl::IterativeClosestPointWithNormals<
        pcl::PointNormal, pcl::PointNormal>                         icp_pl_;
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_;

    // Map
    pcl::PointCloud<pcl::PointXYZI>::Ptr  local_map_;
    pcl::VoxelGrid<pcl::PointXYZI>        map_filter_;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor_;
    pcl::RadiusOutlierRemoval<pcl::PointXYZI>      ror_;

    // State
    Eigen::Matrix4f global_pose_, latest_ekf_pose_, prev_ekf_pose_;
    Eigen::Matrix4f sonar2base_, base2sonar_, ned_transform_;

    std::vector<Keyframe> keyframes_;
    AlgorithmStats        stats_;

    std::mutex ekf_mutex_;
    bool   has_ekf_{false};
    int    has_intensity_{-1};
    int    lost_frames_{0};
    int    reset_counter_{0};
    int    max_lost_frames_{40};
    int    normal_k_search_{20};
    int    submap_size_{20};
    double kf_dist_thresh_{0.5};
    double kf_angle_thresh_{10.0};
    bool   is_ned_{false};
    bool   ekf_z_{false};
    bool   ekf_fallback_{true};
    bool   filter_outliers_{true};
    bool   filter_radius_outliers_{true};
    bool   filter_intensity_{false};
    double min_intensity_{0.0};
    std::string odom_frame_{"odom"};
    std::string base_frame_{"base_link"};
};

// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GicpOdomNode>());
    rclcpp::shutdown();
    return 0;
}
