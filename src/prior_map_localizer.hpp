// MIT License
// Prior Map Localizer with GTSAM — Header
// =========================================
#pragma once

#include <memory>
#include <string>
#include <vector>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <fast_gicp/gicp/fast_vgicp.hpp>
#include <pcl/registration/ndt.h>
#include <Eigen/Geometry>

// ── GTSAM ───────────────────────────────────────────────────
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

using PointT    = pcl::PointXYZI;
using CloudT    = pcl::PointCloud<PointT>;
using CloudTPtr = CloudT::Ptr;

struct Keyframe {
    int             id;
    Eigen::Matrix4f pose;
    CloudTPtr       cloud;
    rclcpp::Time    stamp;
};

class PriorMapLocalizer : public rclcpp::Node
{
public:
    explicit PriorMapLocalizer();

private:
    // ── Callbacks ──────────────────────────────────────────
    void ekfCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void priorMapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void initialPoseCallback(
        const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    // ── Local map building ──────────────────────────────────
    bool buildLocalMap(const CloudTPtr& scan,
                       const Eigen::Matrix4f& ekf_pose,
                       const rclcpp::Time& stamp);
    bool shouldAddKeyframe(const Eigen::Matrix4f& pose);
    void addKeyframe(const CloudTPtr& scan,
                     const Eigen::Matrix4f& pose,
                     const rclcpp::Time& stamp);
    void updateSubmap();

    // ── GTSAM pose graph ────────────────────────────────────
    void addOdometryFactor(int from_id, int to_id,
                            const Eigen::Matrix4f& delta,
                            double fitness_score);
    void addPriorMapFactor(int kf_id,
                            const Eigen::Matrix4f& abs_pose_in_map,
                            double fitness_score);
    void optimizeGraph();
    void updateKeyframePosesFromGraph();

    // ── Global localization ─────────────────────────────────
    void matchLocalMapToPrior(const rclcpp::Time& stamp);

    // ── Helpers ─────────────────────────────────────────────
    CloudTPtr   preprocessCloud(const CloudTPtr& raw);
    void        voxelFilter(CloudTPtr& cloud, float leaf_size);
    CloudTPtr   dbscanFilter(const CloudTPtr& cloud);

    // GTSAM ↔ Eigen
    static gtsam::Pose3     eigenToGtsam(const Eigen::Matrix4f& m);
    static Eigen::Matrix4f  gtsamToEigen(const gtsam::Pose3& p);

    // ── Publishing ──────────────────────────────────────────
    void publishLocalMap(const rclcpp::Time& stamp);
    void publishMapToOdomTF(const rclcpp::Time& stamp);
    void publishMatchMarker(const Eigen::Matrix4f& pose,
                             const rclcpp::Time& stamp,
                             double score);
    void publishOptimizedPath(const rclcpp::Time& stamp);
    void publishFullMap(const rclcpp::Time& stamp);

    // ── ROS interfaces ──────────────────────────────────────
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr                 pc_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr                       ekf_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr                 prior_map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr                    local_map_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr             match_marker_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                              path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr                          vgicp_odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr                    full_map_pub_;
    rclcpp::TimerBase::SharedPtr                                                   tf_timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster>                                 tf_broadcaster_;

    // ── VGICP / NDT ─────────────────────────────────────────
    fast_gicp::FastVGICP<PointT, PointT>        local_vgicp_;
    fast_gicp::FastVGICP<PointT, PointT>        global_vgicp_;
    pcl::NormalDistributionsTransform<PointT, PointT> ndt_;

    // ── GTSAM ───────────────────────────────────────────────
    std::unique_ptr<gtsam::ISAM2>   isam2_;
    gtsam::NonlinearFactorGraph     graph_;
    gtsam::Values                   initial_estimates_;
    bool                            graph_initialized_{false};

    // Noise models
    gtsam::noiseModel::Diagonal::shared_ptr odom_noise_;
    // Prior map noise is built dynamically from fitness score

    // ── Maps / keyframes ────────────────────────────────────
    CloudTPtr             prior_map_;
    CloudTPtr             local_map_;
    std::vector<Keyframe> keyframes_;

    // ── Filters ─────────────────────────────────────────────
    pcl::VoxelGrid<PointT>                map_filter_;
    pcl::StatisticalOutlierRemoval<PointT> sor_filter_;
    pcl::RadiusOutlierRemoval<PointT>     radius_filter_;

    // ── State ───────────────────────────────────────────────
    Eigen::Matrix4f global_pose_      = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f prev_ekf_pose_    = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f ekf_pose_         = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f map_T_odom_       = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f sonar2base_       = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f base2sonar_       = Eigen::Matrix4f::Identity();

    std::mutex ekf_mutex_;
    bool has_ekf_          = false;
    bool has_prior_map_    = false;
    bool has_global_match_ = false;
    bool map_initialized_  = false;
    int  scan_count_       = 0;

    // ── Parameters ──────────────────────────────────────────
    std::string odom_frame_, base_frame_, sonar_frame_, prior_map_frame_;
    std::string pc_topic_, odom_topic_;

    double local_max_dist_, local_epsilon_, local_resolution_;
    int    local_max_iter_;
    double global_max_dist_, global_epsilon_, global_resolution_;
    int    global_max_iter_;
    double score_thresh_;
    int    match_every_n_kf_;

    double kf_dist_thresh_, kf_angle_thresh_;
    int    submap_size_;
    double submap_voxel_res_;

    bool   use_sor_;
    int    sor_neighbors_;
    double sor_stddev_;
    bool   use_radius_;
    double radius_search_;
    int    radius_min_neighbors_;
    bool   use_intensity_filter_;
    float  min_intensity_;

    // DBSCAN cluster filter
    bool   use_dbscan_;
    double dbscan_tolerance_;
    int    dbscan_min_pts_;
    int    dbscan_keep_clusters_;

    // GTSAM noise parameters
    double odom_noise_trans_;
    double odom_noise_rot_;
    double prior_map_noise_scale_;  // multiply fitness score to get noise
    double prior_map_noise_min_;    // minimum noise even for perfect match
    double init_pose_noise_trans_;  // kf0 anchor noise — large = GTSAM can correct init_pose
    double init_pose_noise_rot_;

    // Global matcher selection
    bool   use_ndt_;
    double ndt_resolution_;
    double ndt_step_size_;
    double ndt_epsilon_;
    int    ndt_max_iter_;
};