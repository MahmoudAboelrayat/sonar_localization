#include <memory>
#include <string>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/header.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
class MapNode : public rclcpp::Node
{
public:
    MapNode() : Node("odom_mapping_node")
    {
        // odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("vgicp_odom", 20);
        pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/sonar/point_cloud_local", 20,
            std::bind(&MapNode::pointCloudCallback, this, std::placeholders::_1)
        );
        // odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        //     "/odometry/filtered", 20,
        //     std::bind(&MapNode::odomCallback, this, std::placeholders::_1)
        // );

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/BlueROV2_Heavy/odom_gt", 20,
            std::bind(&MapNode::odomCallback, this, std::placeholders::_1)
        );



        global_pose_ = Eigen::Matrix4f::Identity();
        latest_odom_pose_ = Eigen::Matrix4f::Identity();
        // prev_cloud_odom_pose_ = Eigen::Matrix4f::Identity();

        global_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("odom_global_map", 1);
        global_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

        // Eigen::Vector3f translation_sb(-0.111f, 0.000f, 0.249f);
        // Eigen::Quaternionf rotation_sb(0.966f, -0.000f, 0.259f, -0.000f); 
        Eigen::Vector3f translation_sb(-0.039f, 0.000f, -0.083f);
        Eigen::Quaternionf rotation_sb(0.966f, -0.000f, 0.259f, -0.000f); 
        
        
        sonar2base_ = Eigen::Matrix4f::Identity();
        sonar2base_.block<3,3>(0,0) = rotation_sb.toRotationMatrix();
        sonar2base_.block<3,1>(0,3) = translation_sb;
        base2sonar_ = sonar2base_.inverse();

        RCLCPP_INFO(this->get_logger(), "mapping Node initialized. Waiting for clouds...");
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // std::lock_guard<std::mutex> lock(odom_mutex_);
        
        Eigen::Quaternionf q(
            msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
        Eigen::Vector3f t(
            msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

        latest_odom_pose_.setIdentity();
        latest_odom_pose_.block<3,3>(0,0) = q.toRotationMatrix();
        latest_odom_pose_.block<3,1>(0,3) = t;

        // has_odom_ = true;
    }

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // auto start_time = this->now();

        pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *current_cloud);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_base(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*current_cloud, *cloud_in_base, sonar2base_);
        Eigen::Matrix4f current_odom_pose;
        {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            current_odom_pose = latest_odom_pose_;
        }


        if (first_odom) {
            prev_cloud_odom_pose_ = current_odom_pose;
            // print the first pose
            // Eigen::Vector3f t = prev_cloud_odom_pose_.block<3,1>(0,3);
            // Eigen::Quaternionf q(prev_cloud_odom_pose_.block<3,3>(0,0));
            // RCLCPP_INFO(this->get_logger(), "Current odom pose: position(%.2f, %.2f, %.2f), orientation(%.2f, %.2f, %.2f, %.2f)", 
            //             t.x(), t.y(), t.z(), q.x(), q.y(), q.z(), q.w());   
            first_odom = false;
            return;
        }

        
        Eigen::Matrix4f delta_base = prev_cloud_odom_pose_.inverse() * current_odom_pose;
        

        // global map building:
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        // pcl::transformPointCloud(*current_cloud, *transformed_cloud, latest_odom_pose_*base2sonar_); // Transform using odom pose for better global consistency
        // print the current odom pose
        // Eigen::Vector3f t = delta_base.block<3,1>(0,3);
        // Eigen::Quaternionf q(delta_base.block<3,3>(0,0));
        // RCLCPP_INFO(this->get_logger(), "Current odom pose: position(%.2f, %.2f, %.2f), orientation(%.2f, %.2f, %.2f, %.2f)", 
        //             t.x(), t.y(), t.z(), q.x(), q.y(), q.z(), q.w());   
        pcl::transformPointCloud(*cloud_in_base, *transformed_cloud, current_odom_pose); // Transform using odom pose for better global consistency
        
        // 2. Accumulate into the global map
        *global_map_ += *transformed_cloud;

        // 3. Publish the global map (e.g., every 10 frames to prevent lag)
        static int map_pub_count = 0;
        if (map_pub_count++ % 5 == 0) {
            // print map update time
            // RCLCPP_INFO(this->get_logger(), "Publishing global map with %zu points.", global_map_->size());

            // Optional: Downsample the global map so RViz doesn't crash
            pcl::VoxelGrid<pcl::PointXYZ> vg;
            vg.setInputCloud(global_map_);
            vg.setLeafSize(0.1f, 0.1f, 0.1f); // 10cm resolution
            vg.filter(*global_map_);

            sensor_msgs::msg::PointCloud2 map_msg;
            pcl::toROSMsg(*global_map_, map_msg);
            // map_msg.header.frame_id = "Beckholmen"; // Match your odom frame
            map_msg.header.frame_id = "unity_origin"; // Match your odom frame
            map_msg.header.stamp = msg->header.stamp;
            global_map_pub_->publish(map_msg);
        }


    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr prev_cloud_;
    
    
    Eigen::Matrix4f global_pose_;
    Eigen::Matrix4f latest_odom_pose_;
    Eigen::Matrix4f prev_cloud_odom_pose_;
    Eigen::Matrix4f sonar2base_;
    Eigen::Matrix4f base2sonar_;

    bool first_odom = true;
    std::mutex odom_mutex_;
    // bool has_odom_ = false;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_pub_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapNode>());
    rclcpp::shutdown();
    return 0;
}

