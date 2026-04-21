// MIT License
// Prior Map Localizer — ROS2 C++ Node
// =====================================
// Builds a local submap from sonar scans, matches it against a
// pre-built prior map, and publishes an absolute pose correction
// that feeds into the existing robot_localization EKF.
//
// Topics:
//   Subscribes:
//     /sonar/point_cloud_intensity  (sensor_msgs/PointCloud2)
//     /odometry/filtered            (nav_msgs/Odometry) — EKF dead reckoning
//   Publishes:
//     /prior_map_pose               (nav_msgs/Odometry) — absolute pose for EKF
//     /local_map                    (sensor_msgs/PointCloud2) — current submap
//     /prior_map                    (sensor_msgs/PointCloud2) — loaded prior map

#pragma once

#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <deque>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <fast_gicp/gicp/fast_vgicp.hpp>
#include <Eigen/Geometry>

using PointT    = pcl::PointXYZI;
using CloudT    = pcl::PointCloud<PointT>;
using CloudTPtr = CloudT::Ptr;

// ── Keyframe ────────────────────────────────────────────────
struct Keyframe {
    int              id;
    Eigen::Matrix4f  pose;    // pose in odom frame when keyframe was added
    CloudTPtr        cloud;   // scan in base_link frame
    rclcpp::Time     stamp;
};

// ── Node ────────────────────────────────────────────────────
class PriorMapLocalizer : public rclcpp::Node
{
public:
    explicit PriorMapLocalizer();

private:
    // ── Callbacks ──────────────────────────────────────────
    void ekfCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    // ── Local map building ──────────────────────────────────
    bool        buildLocalMap(const CloudTPtr& scan,
                               const Eigen::Matrix4f& ekf_pose,
                               const rclcpp::Time& stamp);
    bool        shouldAddKeyframe(const Eigen::Matrix4f& pose);
    void        addKeyframe(const CloudTPtr& scan,
                             const Eigen::Matrix4f& pose,
                             const rclcpp::Time& stamp);
    void        updateSubmap();

    // ── Global localization ─────────────────────────────────
    void        matchLocalMapToPrior(const rclcpp::Time& stamp);

    // ── Preprocessing ───────────────────────────────────────
    CloudTPtr   preprocessCloud(const CloudTPtr& raw);
    void        voxelFilter(CloudTPtr& cloud, float leaf_size);

    // ── Publishing ──────────────────────────────────────────
    void        publishLocalMap(const rclcpp::Time& stamp);
    void        publishPriorMap(const rclcpp::Time& stamp);
    void        publishMatchMarker(const Eigen::Matrix4f& pose,
                                    const rclcpp::Time& stamp,
                                    double score);
    void        publishMapToOdomTF(const rclcpp::Time& stamp);

    // ── ROS interfaces ──────────────────────────────────────
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr           pc_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr                 ekf_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    local_map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    prior_map_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr match_marker_pub_;
    rclcpp::TimerBase::SharedPtr                                   prior_map_timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster>                 tf_broadcaster_;

    // ── VGICP instances ─────────────────────────────────────
    fast_gicp::FastVGICP<PointT, PointT> local_vgicp_;   // scan → submap
    fast_gicp::FastVGICP<PointT, PointT> global_vgicp_;  // submap → prior

    // ── Maps ────────────────────────────────────────────────
    CloudTPtr prior_map_;
    CloudTPtr local_map_;
    std::vector<Keyframe> keyframes_;

    // ── Filters ─────────────────────────────────────────────
    pcl::VoxelGrid<PointT>               map_filter_;
    pcl::StatisticalOutlierRemoval<PointT> sor_filter_;
    pcl::RadiusOutlierRemoval<PointT>    radius_filter_;

    // ── State ───────────────────────────────────────────────
    Eigen::Matrix4f global_pose_      = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f prev_ekf_pose_    = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f ekf_pose_         = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f last_global_pose_ = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f sonar2base_       = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f base2sonar_       = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f map_T_odom_       = Eigen::Matrix4f::Identity(); // map → odom correction


    std::mutex ekf_mutex_;
    bool has_ekf_          = false;
    bool has_global_match_ = false;
    bool map_initialized_  = false;
    int  scan_count_       = 0;

    // ── Parameters ──────────────────────────────────────────
    std::string odom_frame_;
    std::string prior_map_frame_;
    std::string base_frame_;
    std::string sonar_frame_;

    std::string pc_topic_;
    std::string odom_topic_;

    // Local VGICP
    double local_max_dist_;
    double local_epsilon_;
    int    local_max_iter_;
    double local_resolution_;

    // Global VGICP
    double global_max_dist_;
    double global_epsilon_;
    int    global_max_iter_;
    double global_resolution_;
    double score_thresh_;
    int    match_every_n_kf_;

    double prior_map_resolution_;

    // Keyframe / submap
    double kf_dist_thresh_;
    double kf_angle_thresh_;
    int    submap_size_;

    // Outlier removal
    bool   use_sor_;
    int    sor_neighbors_;
    double sor_stddev_;
    bool   use_radius_;
    double radius_search_;
    int    radius_min_neighbors_;

    // Intensity filter
    bool  use_intensity_filter_;
    float min_intensity_;
};