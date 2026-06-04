#include <memory>
#include <string>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include <Eigen/Geometry>

class MapNode : public rclcpp::Node
{
public:
    MapNode() : Node("odom_mapping_node")
    {
        // ── Parameters ────────────────────────────────────────────────────────
        std::string pc_topic   = this->declare_parameter<std::string>("pc_topic",   "usv/point_cloud");
        std::string odom_topic = this->declare_parameter<std::string>("odom_topic", "/imu/odometry");
        std::string map_topic  = this->declare_parameter<std::string>("map_topic",  "/prior_map");
        odom_frame_            = this->declare_parameter<std::string>("odom_frame", "odom");
        map_resolution_        = static_cast<float>(this->declare_parameter<double>("map_resolution", 0.05));
        publish_every_         = this->declare_parameter<int>("publish_every", 5);

        // sonar → base_link extrinsic (translation in metres, rotation as quaternion w,x,y,z)
        double tx = this->declare_parameter<double>("sonar2base_tx",  -0.056);
        double ty = this->declare_parameter<double>("sonar2base_ty",   0.000);
        double tz = this->declare_parameter<double>("sonar2base_tz",  -0.311);
        double qw = this->declare_parameter<double>("sonar2base_qw",   0.966);
        double qx = this->declare_parameter<double>("sonar2base_qx",  -0.000);
        double qy = this->declare_parameter<double>("sonar2base_qy",   0.259);
        double qz = this->declare_parameter<double>("sonar2base_qz",  -0.000);

        Eigen::Quaternionf q(static_cast<float>(qw), static_cast<float>(qx),
                             static_cast<float>(qy), static_cast<float>(qz));
        q.normalize();
        sonar2base_.setIdentity();
        sonar2base_.block<3,3>(0,0) = q.toRotationMatrix();
        sonar2base_.block<3,1>(0,3) = Eigen::Vector3f(tx, ty, tz);
        base2sonar_ = sonar2base_.inverse();

        RCLCPP_INFO(get_logger(),
            "sonar2base: t=[%.3f, %.3f, %.3f]  q=[%.3f, %.3f, %.3f, %.3f]",
            tx, ty, tz, qw, qx, qy, qz);

        // ── Subscriptions ─────────────────────────────────────────────────────
        pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            pc_topic, 20,
            std::bind(&MapNode::pointCloudCallback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 20,
            std::bind(&MapNode::odomCallback, this, std::placeholders::_1));

        // ── Publisher ─────────────────────────────────────────────────────────
        rclcpp::QoS map_qos(1);
        global_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(map_topic, map_qos);
        global_map_     = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

        RCLCPP_INFO(get_logger(), "MapNode ready — pc: %s  odom: %s  map: %s",
            pc_topic.c_str(), odom_topic.c_str(), map_topic.c_str());
    }

private:
    // ── Odometry callback ─────────────────────────────────────────────────────
    // Stores T_odom_base: pose of base_link expressed in odom frame.
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        Eigen::Quaternionf q(
            static_cast<float>(msg->pose.pose.orientation.w),
            static_cast<float>(msg->pose.pose.orientation.x),
            static_cast<float>(msg->pose.pose.orientation.y),
            static_cast<float>(msg->pose.pose.orientation.z));
        Eigen::Vector3f t(
            static_cast<float>(msg->pose.pose.position.x),
            static_cast<float>(msg->pose.pose.position.y),
            static_cast<float>(msg->pose.pose.position.z));

        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        pose.block<3,3>(0,0) = q.normalized().toRotationMatrix();
        pose.block<3,1>(0,3) = t;

        std::lock_guard<std::mutex> lock(odom_mutex_);
        latest_odom_pose_ = pose;
        has_odom_ = true;
    }

    // ── Point cloud callback ──────────────────────────────────────────────────
    // Transforms each incoming sonar-frame cloud into the odom frame and
    // accumulates it into the global map.
    //
    // Transform chain:  p_odom = T_odom_base * T_base_sonar * p_sonar
    //                          = latest_odom_pose_  *  sonar2base_  *  p_sonar
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Grab odom pose under lock
        Eigen::Matrix4f T_odom_base;
        {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            if (!has_odom_) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "Waiting for first odometry message...");
                return;
            }
            T_odom_base = latest_odom_pose_;
        }

        // Decode incoming cloud (sonar frame)
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sonar(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud_sonar);

        if (cloud_sonar->empty()) return;

        // Transform: sonar → odom
        // T_odom_sonar = T_odom_base * T_base_sonar = latest_odom_pose_ * sonar2base_
        Eigen::Matrix4f T_odom_sonar = T_odom_base * base2sonar_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_odom(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*cloud_sonar, *cloud_odom, T_odom_sonar);

        // Accumulate into global map
        *global_map_ += *cloud_odom;

        // Publish (downsampled) every N frames
        if (pub_count_++ % publish_every_ == 0) {
            pcl::VoxelGrid<pcl::PointXYZ> vg;
            vg.setInputCloud(global_map_);
            vg.setLeafSize(map_resolution_, map_resolution_, map_resolution_);
            vg.filter(*global_map_);

            sensor_msgs::msg::PointCloud2 map_msg;
            pcl::toROSMsg(*global_map_, map_msg);
            map_msg.header.frame_id = odom_frame_;
            map_msg.header.stamp    = msg->header.stamp;
            global_map_pub_->publish(map_msg);

            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                "Global map: %zu points", global_map_->size());
        }
    }

    // ── Members ───────────────────────────────────────────────────────────────
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr        odom_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr     global_map_pub_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_;

    std::mutex       odom_mutex_;
    Eigen::Matrix4f  latest_odom_pose_{Eigen::Matrix4f::Identity()};
    bool             has_odom_{false};

    Eigen::Matrix4f  sonar2base_{Eigen::Matrix4f::Identity()};  // T_base_sonar
    Eigen::Matrix4f  base2sonar_{Eigen::Matrix4f::Identity()};  // T_sonar_base = T_base_sonar^-1

    std::string odom_frame_{"odom"};
    float       map_resolution_{0.2f};
    int         publish_every_{5};
    int         pub_count_{0};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapNode>());
    rclcpp::shutdown();
    return 0;
}
