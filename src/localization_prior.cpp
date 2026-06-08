// localization_prior.cpp
//
// Builds a local map from EKF odometry + sonar point clouds.
// Keeps a sliding window of the last N keyframes and publishes the
// accumulated map for the map_matcher node to align against the prior map.

#include <memory>
#include <string>
#include <mutex>
#include <vector>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/common/transforms.h>

#include <Eigen/Geometry>

struct Keyframe {
    Eigen::Matrix4f pose;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
};

class LocalizationPriorNode : public rclcpp::Node
{
public:
    LocalizationPriorNode(const rclcpp::NodeOptions & options)
        : Node("localization_prior_node", options)
    {
        // ── Parameters ────────────────────────────────────────────────────────
        std::string pc_topic   = declare_parameter<std::string>("topics.pc_sub",             "/sonar/point_cloud");
        std::string ekf_topic  = declare_parameter<std::string>("topics.ekf_sub",            "/odometry/filtered");
        std::string map_topic  = declare_parameter<std::string>("topics.map_pub",            "local_map");
        std::string odom_topic = declare_parameter<std::string>("topics.odom_pub",           "vgicp_odom");
        std::string path_topic = declare_parameter<std::string>("topics.path_pub",           "vgicp_path");
        odom_frame_   = declare_parameter<std::string>("frames.odom_frame", "icp_map");
        base_frame_   = declare_parameter<std::string>("frames.base_frame", "dvl_link");

        max_keyframes_   = declare_parameter<int>   ("keyframe.max_keyframes",  20);
        kf_dist_thresh_  = declare_parameter<double>("keyframe.dist_thresh",    0.5);
        kf_angle_thresh_ = declare_parameter<double>("keyframe.angle_thresh",   8.0);
        map_res_         = declare_parameter<float> ("map_resolution",           0.05f);
        publish_every_   = declare_parameter<int>   ("publish_every",            5);
        ekf_max_age_     = declare_parameter<double>("ekf_max_age",              0.2);

        double b2s_x     = declare_parameter<double>("tf.base2sonar_x",     0.163);
        double b2s_y     = declare_parameter<double>("tf.base2sonar_y",     0.1437);
        double b2s_z     = declare_parameter<double>("tf.base2sonar_z",     -0.3794);
        double b2s_roll  = declare_parameter<double>("tf.base2sonar_roll",  0.0);
        double b2s_pitch = declare_parameter<double>("tf.base2sonar_pitch", -20);
        double b2s_yaw   = declare_parameter<double>("tf.base2sonar_yaw",   0.0);

        Eigen::Quaternionf q_sb;
        q_sb = Eigen::AngleAxisf(static_cast<float>(b2s_yaw   * M_PI/180.0), Eigen::Vector3f::UnitZ())
             * Eigen::AngleAxisf(static_cast<float>(b2s_pitch * M_PI/180.0), Eigen::Vector3f::UnitY())
             * Eigen::AngleAxisf(static_cast<float>(b2s_roll  * M_PI/180.0), Eigen::Vector3f::UnitX());
        base2sonar_ = Eigen::Matrix4f::Identity();
        base2sonar_.block<3,3>(0,0) = q_sb.toRotationMatrix();
        base2sonar_.block<3,1>(0,3) = Eigen::Vector3f(b2s_x, b2s_y, b2s_z);

        // Outlier removal (optional)
        filter_sor_ = declare_parameter<bool>  ("outlier_removal.filter_outliers",   false);
        filter_ror_ = declare_parameter<bool>  ("radius_outlier_removal.filter_radius_outliers", false);
        sor_.setMeanK(declare_parameter<int>("outlier_removal.num_neighbors", 30));
        sor_.setStddevMulThresh(declare_parameter<double>("outlier_removal.stddev_mul_thresh", 0.7));
        ror_.setRadiusSearch(declare_parameter<double>("radius_outlier_removal.search_radius", 0.2));
        ror_.setMinNeighborsInRadius(declare_parameter<int>("radius_outlier_removal.min_neighbors_in_radius", 30));

        map_filter_.setLeafSize(map_res_, map_res_, map_res_);

        // ── Subscriptions ─────────────────────────────────────────────────────
        auto ekf_cb = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto pc_cb  = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions ekf_opt, pc_opt;
        ekf_opt.callback_group = ekf_cb;
        pc_opt.callback_group  = pc_cb;

        ekf_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            ekf_topic, 10,
            std::bind(&LocalizationPriorNode::ekfCallback, this, std::placeholders::_1),
            ekf_opt);

        pc_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            pc_topic, 10,
            std::bind(&LocalizationPriorNode::pointCloudCallback, this, std::placeholders::_1),
            pc_opt);

        // ── Publishers ─────────────────────────────────────────────────────────
        // Use transient_local (latched) + reliable so RViz always gets the latest map
        rclcpp::QoS map_qos(1);
        map_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
        map_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
        map_pub_  = create_publisher<sensor_msgs::msg::PointCloud2>(map_topic,  map_qos);
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>      (odom_topic, 10);
        path_pub_ = create_publisher<nav_msgs::msg::Path>          (path_topic, 10);

        RCLCPP_INFO(get_logger(),
            "localization_prior ready — ekf: %s  pc: %s  max_kf: %d",
            ekf_topic.c_str(), pc_topic.c_str(), max_keyframes_);
    }

private:
    // ── EKF callback ──────────────────────────────────────────────────────────
    void ekfCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        Eigen::Quaternionf q(
            msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
        Eigen::Vector3f t(
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z);

        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        pose.block<3,3>(0,0) = q.normalized().toRotationMatrix();
        pose.block<3,1>(0,3) = t;

        std::lock_guard<std::mutex> lk(ekf_mutex_);
        latest_ekf_pose_  = pose;
        latest_ekf_stamp_ = msg->header.stamp;
        has_ekf_ = true;
    }

    // ── Point cloud callback ───────────────────────────────────────────────────
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Get EKF pose snapshot
        Eigen::Matrix4f pose;
        rclcpp::Time    ekf_stamp;
        {
            std::lock_guard<std::mutex> lk(ekf_mutex_);
            if (!has_ekf_) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "Waiting for EKF odometry on %s", ekf_sub_->get_topic_name());
                return;
            }
            double age = std::abs(
                (rclcpp::Time(msg->header.stamp) - rclcpp::Time(latest_ekf_stamp_)).seconds());
            if (age > ekf_max_age_) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "EKF too old (%.3fs) — skipping scan", age);
                return;
            }
            pose      = latest_ekf_pose_;
            ekf_stamp = latest_ekf_stamp_;
        }

        // Check keyframe threshold
        {
            std::lock_guard<std::mutex> lk(kf_mutex_);
            if (!keyframes_.empty()) {
                Eigen::Matrix4f delta = keyframes_.back().pose.inverse() * pose;
                float dist  = delta.block<3,1>(0,3).norm();
                float angle = Eigen::AngleAxisf(
                    Eigen::Matrix3f(delta.block<3,3>(0,0))).angle() * 180.0f / M_PI;
                if (dist < static_cast<float>(kf_dist_thresh_) &&
                    angle < static_cast<float>(kf_angle_thresh_))
                    return;
            }
        }

        // Convert and filter the scan
        pcl::PointCloud<pcl::PointXYZ>::Ptr raw(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *raw);

        // Transform sonar → base_link
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_base(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*raw, *cloud_base, base2sonar_);

        // Optional outlier removal
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered = cloud_base;
        if (filter_ror_) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
            ror_.setInputCloud(filtered);
            ror_.filter(*tmp);
            filtered = tmp;
        }
        if (filter_sor_) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
            sor_.setInputCloud(filtered);
            sor_.filter(*tmp);
            filtered = tmp;
        }
        if (filtered->empty()) return;

        // Add keyframe
        {
            std::lock_guard<std::mutex> lk(kf_mutex_);
            Keyframe kf;
            kf.pose  = pose;
            kf.cloud = filtered;
            keyframes_.push_back(kf);

            // Sliding window — drop oldest
            if (static_cast<int>(keyframes_.size()) > max_keyframes_)
                keyframes_.erase(keyframes_.begin());
        }

        // Publish odometry (passthrough from EKF, in odom_frame)
        publishOdometry(msg->header.stamp, pose);

        // Publish path
        publishPath(msg->header.stamp);

        // Publish local map every N scans
        if (++pub_count_ % publish_every_ == 0)
            publishMap(msg->header.stamp);
    }

    // ── Publish local map ─────────────────────────────────────────────────────
    void publishMap(const rclcpp::Time & stamp)
    {
        pcl::PointCloud<pcl::PointXYZ> full;
        {
            std::lock_guard<std::mutex> lk(kf_mutex_);
            for (auto & kf : keyframes_) {
                pcl::PointCloud<pcl::PointXYZ> tmp;
                pcl::transformPointCloud(*kf.cloud, tmp, kf.pose);
                full += tmp;
            }
        }
        if (full.empty()) return;

        pcl::PointCloud<pcl::PointXYZ>::Ptr ds(new pcl::PointCloud<pcl::PointXYZ>);
        map_filter_.setInputCloud(full.makeShared());
        map_filter_.filter(*ds);

        sensor_msgs::msg::PointCloud2 map_msg;
        pcl::toROSMsg(*ds, map_msg);
        map_msg.header.frame_id = odom_frame_;
        map_msg.header.stamp    = stamp;
        map_pub_->publish(map_msg);
    }

    // ── Publish odometry ──────────────────────────────────────────────────────
    void publishOdometry(const rclcpp::Time & stamp, const Eigen::Matrix4f & pose)
    {
        Eigen::Vector3f    t(pose.block<3,1>(0,3));
        Eigen::Quaternionf q(pose.block<3,3>(0,0));

        nav_msgs::msg::Odometry odom;
        odom.header.stamp    = stamp;
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

    // ── Publish path ──────────────────────────────────────────────────────────
    void publishPath(const rclcpp::Time & stamp)
    {
        nav_msgs::msg::Path path;
        path.header.frame_id = odom_frame_;
        path.header.stamp    = stamp;
        {
            std::lock_guard<std::mutex> lk(kf_mutex_);
            for (auto & kf : keyframes_) {
                geometry_msgs::msg::PoseStamped ps;
                ps.header = path.header;
                Eigen::Vector3f    t(kf.pose.block<3,1>(0,3));
                Eigen::Quaternionf q(kf.pose.block<3,3>(0,0));
                ps.pose.position.x    = t.x(); ps.pose.position.y    = t.y(); ps.pose.position.z    = t.z();
                ps.pose.orientation.x = q.x(); ps.pose.orientation.y = q.y();
                ps.pose.orientation.z = q.z(); ps.pose.orientation.w = q.w();
                path.poses.push_back(ps);
            }
        }
        path_pub_->publish(path);
    }

    // ── Members ───────────────────────────────────────────────────────────────
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr        ekf_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr  pc_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr     map_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr           odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr               path_pub_;

    std::mutex      ekf_mutex_;
    Eigen::Matrix4f latest_ekf_pose_{Eigen::Matrix4f::Identity()};
    rclcpp::Time    latest_ekf_stamp_{0, 0, RCL_ROS_TIME};
    bool            has_ekf_{false};

    std::mutex              kf_mutex_;
    std::vector<Keyframe>   keyframes_;

    Eigen::Matrix4f base2sonar_{Eigen::Matrix4f::Identity()};
    pcl::VoxelGrid<pcl::PointXYZ>                map_filter_;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor_;
    pcl::RadiusOutlierRemoval<pcl::PointXYZ>      ror_;
    bool filter_sor_{false}, filter_ror_{false};

    std::string odom_frame_{"icp_map"}, base_frame_{"dvl_link"};
    int    max_keyframes_{30};
    double kf_dist_thresh_{0.3}, kf_angle_thresh_{5.0};
    float  map_res_{0.1f};
    int    publish_every_{5}, pub_count_{0};
    double ekf_max_age_{0.2};
};

// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<LocalizationPriorNode>(options);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
