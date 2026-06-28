// map_matcher.cpp
//
// Subscribes to a pre-built global/prior map and the SLAM-built local map.
// Periodically aligns the SLAM map onto the global map with VGICP and
// publishes the correction to /initialpose so odom_tf.py updates the
// world → icp_map static TF.

#include <memory>
#include <string>
#include <mutex>
#include <thread>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <fast_gicp/gicp/fast_vgicp.hpp>
#include <pcl/registration/ndt.h>
#include <Eigen/Geometry>

class MapMatcherNode : public rclcpp::Node
{
public:
    MapMatcherNode(const rclcpp::NodeOptions & options)
        : Node("map_matcher_node", options)
    {
        // ── Parameters ────────────────────────────────────────────────────────
        global_map_topic_  = declare_parameter<std::string>("global_map_topic",  "/prior_map");
        slam_map_topic_    = declare_parameter<std::string>("slam_map_topic",     "vgicp_full_map");
        initialpose_topic_ = declare_parameter<std::string>("initialpose_topic",  "/initialpose");
        child_frame_       = declare_parameter<std::string>("child_frame",        "icp_map");
        period_            = declare_parameter<double>("period",            5.0);
        min_keyframes_     = declare_parameter<int>   ("min_keyframes",     0);
        voxel_size_        = declare_parameter<float> ("voxel_size",        0.3f);
        fitness_threshold_ = declare_parameter<double>("fitness_threshold", 0.3);
        max_corr_dist_     = declare_parameter<double>("max_corr_dist",     2.0);
        use_ndt_           = declare_parameter<bool>  ("use_ndt",           false);
        is_ned_            = declare_parameter<bool>  ("is_ned",            false);
        int vgicp_threads  = declare_parameter<int>   ("vgicp_threads",     4);
        int vgicp_max_iter = declare_parameter<int>   ("vgicp_max_iter",    150);
        double vgicp_res   = declare_parameter<double>("vgicp_resolution",  0.3);
        double vgicp_eps   = declare_parameter<double>("vgicp_epsilon",     1e-3);
        double ndt_res     = declare_parameter<double>("ndt_resolution",    1.0);
        double ndt_step    = declare_parameter<double>("ndt_step_size",     0.5);
        double ndt_eps     = declare_parameter<double>("ndt_epsilon",       0.01);
        int    ndt_iter    = declare_parameter<int>   ("ndt_max_iter",      50);

        // ── VGICP ─────────────────────────────────────────────────────────────
        vgicp_.setNumThreads(vgicp_threads);
        vgicp_.setMaximumIterations(vgicp_max_iter);
        vgicp_.setResolution(vgicp_res);
        vgicp_.setTransformationEpsilon(vgicp_eps);
        vgicp_.setMaxCorrespondenceDistance(max_corr_dist_);

        // ── NDT ───────────────────────────────────────────────────────────────
        ndt_.setResolution(static_cast<float>(ndt_res));
        ndt_.setStepSize(ndt_step);
        ndt_.setTransformationEpsilon(ndt_eps);
        ndt_.setMaximumIterations(ndt_iter);

        // NED → SLAM-frame transform (same convention as localization_imu_pre)
        ned_transform_ << 0, -1,  0, 0,
                          1,  0,  0, 0,
                          0,  0, -1, 0,
                          0,  0,  0, 1;

        RCLCPP_INFO(get_logger(), "Matcher: %s  is_ned: %s",
            use_ndt_ ? "NDT" : "VGICP", is_ned_ ? "true" : "false");

        // ── Subscriptions ─────────────────────────────────────────────────────
        // Global map — latched so it arrives even if published before we start
        rclcpp::QoS latched_qos(1);
        latched_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
        latched_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);

        global_map_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            global_map_topic_, latched_qos,
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
                pcl::fromROSMsg(*msg, *cloud);
                // Prior map is in NED frame; SLAM runs in non-NED → convert
                if (!is_ned_) {
                    auto converted = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
                    pcl::transformPointCloud(*cloud, *converted, ned_transform_);
                    cloud = converted;
                }
                cloud = downsample(cloud);
                std::lock_guard<std::mutex> lk(global_mutex_);
                global_map_ = cloud;
                has_global_ = true;
                RCLCPP_INFO(get_logger(), "Global map received: %zu pts (is_ned=%s)",
                    cloud->size(), is_ned_ ? "true" : "false");
            });

        rclcpp::QoS be_qos(1);
        be_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);

        slam_map_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            slam_map_topic_, be_qos,
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
                pcl::fromROSMsg(*msg, *cloud);
                cloud = downsample(cloud);
                std::lock_guard<std::mutex> lk(slam_mutex_);
                slam_map_ = cloud;
                has_slam_ = true;
                ++slam_kf_count_;
            });

        // ── Publisher ─────────────────────────────────────────────────────────
        initialpose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            initialpose_topic_, 1);

        // ── Matching thread ───────────────────────────────────────────────────
        match_thread_ = std::thread(&MapMatcherNode::matchThread, this);

        RCLCPP_INFO(get_logger(),
            "map_matcher ready  global=%s  slam=%s  period=%.1fs",
            global_map_topic_.c_str(), slam_map_topic_.c_str(), period_);
    }

    ~MapMatcherNode()
    {
        running_ = false;
        if (match_thread_.joinable()) match_thread_.join();
    }

private:
    // ── Helpers ───────────────────────────────────────────────────────────────
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsample(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr ds(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
        vg.setInputCloud(cloud);
        vg.filter(*ds);
        return ds;
    }

    // ── Matching thread ───────────────────────────────────────────────────────
    void matchThread()
    {
        while (rclcpp::ok() && running_) {
            std::this_thread::sleep_for(std::chrono::duration<double>(period_));
            performMatch();
        }
    }

    void performMatch()
    {
        // Snapshot both maps
        pcl::PointCloud<pcl::PointXYZ>::Ptr global_snap, slam_snap;
        {
            std::lock_guard<std::mutex> lk(global_mutex_);
            if (!has_global_) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                    "Waiting for global map on %s", global_map_topic_.c_str());
                return;
            }
            global_snap = global_map_;
        }
        {
            std::lock_guard<std::mutex> lk(slam_mutex_);
            if (!has_slam_ || slam_map_->size() < 20) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                    "Waiting for SLAM map on %s", slam_map_topic_.c_str());
                return;
            }
            if (min_keyframes_ > 0 && slam_kf_count_ < min_keyframes_) {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                    "Waiting for keyframes: %d / %d", slam_kf_count_.load(), min_keyframes_);
                return;
            }
            slam_snap = slam_map_;
        }

        // Align SLAM map (source) → global map (target)
        pcl::PointCloud<pcl::PointXYZ> aligned;
        bool converged; double score; Eigen::Matrix4f T;
        const char* matcher = use_ndt_ ? "NDT" : "VGICP";
        {
            std::lock_guard<std::mutex> lk(correction_mutex_);
            if (use_ndt_) {
                ndt_.setInputSource(slam_snap);
                ndt_.setInputTarget(global_snap);
                ndt_.align(aligned, last_correction_);
                converged = ndt_.hasConverged();
                score     = ndt_.getFitnessScore();
                T         = ndt_.getFinalTransformation();
            } else {
                vgicp_.setInputSource(slam_snap);
                vgicp_.setInputTarget(global_snap);
                vgicp_.align(aligned, last_correction_);
                converged = vgicp_.hasConverged();
                score     = vgicp_.getFitnessScore();
                T         = vgicp_.getFinalTransformation();
            }
        }

        Eigen::Vector3f rpy = T.block<3,3>(0,0).cast<float>().eulerAngles(0, 1, 2)
                              * (180.0f / M_PI);
        RCLCPP_INFO(get_logger(),
            "%s  converged=%d  score=%.4f  "
            "t=[%.2f, %.2f, %.2f] m  rpy=[%.1f, %.1f, %.1f] deg  "
            "src=%zu  tgt=%zu",
            matcher, converged, score,
            T(0,3), T(1,3), T(2,3),
            rpy(0), rpy(1), rpy(2),
            slam_snap->size(), global_snap->size());

        if (!converged) {
            RCLCPP_WARN(get_logger(), "%s did not converge", matcher);
            return;
        }
        if (score > fitness_threshold_) {
            RCLCPP_WARN(get_logger(),
                "ICP refused — score %.4f > threshold %.4f", score, fitness_threshold_);
            return;
        }

        {
            std::lock_guard<std::mutex> lk(correction_mutex_);
            last_correction_ = T;
        }

        publishCorrection(T);
        RCLCPP_WARN(get_logger(),
            "Correction accepted!  score=%.4f  t=[%.2f,%.2f,%.2f]",
            score, T(0,3), T(1,3), T(2,3));
    }

    void publishCorrection(const Eigen::Matrix4f & T)
    {
        // T maps SLAM-frame (icp_map) points into the global/world frame.
        // Its translation+rotation is exactly T_{world→icp_map} that odom_tf.py needs.
        Eigen::Quaternionf q(T.block<3,3>(0,0));
        q.normalize();

        geometry_msgs::msg::PoseWithCovarianceStamped msg;
        msg.header.stamp    = this->now();
        msg.header.frame_id = child_frame_;   // signals odom_tf.py: direct TF, not RViz click
        msg.pose.pose.position.x    = T(0,3);
        msg.pose.pose.position.y    = T(1,3);
        msg.pose.pose.position.z    = T(2,3);
        msg.pose.pose.orientation.x = q.x();
        msg.pose.pose.orientation.y = q.y();
        msg.pose.pose.orientation.z = q.z();
        msg.pose.pose.orientation.w = q.w();
        msg.pose.covariance[0]  = 0.1;
        msg.pose.covariance[7]  = 0.1;
        msg.pose.covariance[14] = 0.1;
        msg.pose.covariance[35] = 0.1;
        initialpose_pub_->publish(msg);
    }

    // ── Members ───────────────────────────────────────────────────────────────
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_sub_, slam_map_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_pub_;
    std::thread match_thread_;
    std::atomic<bool> running_{true};

    fast_gicp::FastVGICP<pcl::PointXYZ, pcl::PointXYZ>          vgicp_;
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_;
    bool            use_ndt_{false};
    bool            is_ned_{false};
    Eigen::Matrix4f ned_transform_;

    std::mutex global_mutex_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_;
    bool has_global_{false};

    std::mutex slam_mutex_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr slam_map_;
    bool has_slam_{false};
    std::atomic<int> slam_kf_count_{0};
    int min_keyframes_{0};

    std::mutex      correction_mutex_;
    Eigen::Matrix4f last_correction_{Eigen::Matrix4f::Identity()};

    std::string global_map_topic_, slam_map_topic_, initialpose_topic_, child_frame_;
    double period_{5.0}, fitness_threshold_{0.3}, max_corr_dist_{2.0};
    float  voxel_size_{0.3f};
};

// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<MapMatcherNode>(options);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
