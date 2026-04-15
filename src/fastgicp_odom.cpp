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
#include <pcl/filters/voxel_grid.h>

#include <fast_gicp/gicp/fast_gicp.hpp>
#include <Eigen/Geometry>

class GicpOdomNode : public rclcpp::Node
{
public:
    GicpOdomNode() : Node("fast_gicp_odom_node")
    {
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 20);
        pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/sonar/point_cloud", 10,
            std::bind(&GicpOdomNode::pointCloudCallback, this, std::placeholders::_1)
        );
        ekf_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odometry/odometry/ekf_local", 10,
            std::bind(&GicpOdomNode::ekfCallback, this, std::placeholders::_1)
        );

        global_pose_ = Eigen::Matrix4f::Identity();
        latest_ekf_pose_ = Eigen::Matrix4f::Identity();
        prev_cloud_ekf_pose_ = Eigen::Matrix4f::Identity();

        Eigen::Vector3f translation_sb(-0.111f, 0.000f, 0.249f);
        Eigen::Quaternionf rotation_sb(0.966f, -0.000f, 0.259f, -0.000f); 
        
        sonar2base_ = Eigen::Matrix4f::Identity();
        sonar2base_.block<3,3>(0,0) = rotation_sb.toRotationMatrix();
        sonar2base_.block<3,1>(0,3) = translation_sb;
        base2sonar_ = sonar2base_.inverse();

        gicp_.setNumThreads(4);               
        gicp_.setTransformationEpsilon(1e-6);
        gicp_.setMaxCorrespondenceDistance(0.2);
        gicp_.setMaximumIterations(30);      

        RCLCPP_INFO(this->get_logger(), "FastGICP Odometry Node initialized. Waiting for clouds...");
    }

private:
    void ekfCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(ekf_mutex_);
        
        Eigen::Quaternionf q(
            msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
        Eigen::Vector3f t(
            msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

        latest_ekf_pose_.setIdentity();
        latest_ekf_pose_.block<3,3>(0,0) = q.toRotationMatrix();
        latest_ekf_pose_.block<3,1>(0,3) = t;
        
        has_ekf_ = true;
    }


    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        auto start_time = this->now();

        // 1. Create a clean PCL cloud and reserve memory to prevent re-allocations
        pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        int num_points = msg->width * msg->height;
        current_cloud->points.reserve(num_points);

        // 2. Manually parse the 13-byte packed data
        // msg->data is a vector of uint8_t
        const uint8_t* ptr = msg->data.data();
        uint32_t point_step = msg->point_step; // This is 13 in your case

        for (size_t i = 0; i < num_points; ++i) {
            // Calculate the starting position of this point in the byte array
            const uint8_t* point_ptr = ptr + (i * point_step);
            
            // Use memcpy to safely extract floats even if memory is unaligned
            pcl::PointXYZ p;
            std::memcpy(&p.x, point_ptr + 0, sizeof(float)); // Offset 0
            std::memcpy(&p.y, point_ptr + 4, sizeof(float)); // Offset 4
            std::memcpy(&p.z, point_ptr + 8, sizeof(float)); // Offset 8

            // Only add valid points to the cloud
            if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
                current_cloud->points.push_back(p);
            }
        }
        
        current_cloud->width = current_cloud->points.size();
        current_cloud->height = 1;
        current_cloud->is_dense = true;

        // 3. Continue with EKF and Alignment logic...
        Eigen::Matrix4f current_ekf_pose;
        {
            std::lock_guard<std::mutex> lock(ekf_mutex_);
            current_ekf_pose = latest_ekf_pose_;
        }

        if (!prev_cloud_) {
            prev_cloud_ = current_cloud;
            prev_cloud_ekf_pose_ = current_ekf_pose;
            return;
        }
        
       
    // void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    // {
    //     auto start_time = this->now();

    //     pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //     pcl::fromROSMsg(*msg, *current_cloud);

    //     Eigen::Matrix4f current_ekf_pose;
    //     {
    //         std::lock_guard<std::mutex> lock(ekf_mutex_);
    //         current_ekf_pose = latest_ekf_pose_;
    //     }

    //     if (!prev_cloud_) {
    //         prev_cloud_ = current_cloud;
    //         prev_cloud_ekf_pose_ = current_ekf_pose;
    //         return;
    //     }

        gicp_.setInputTarget(prev_cloud_);
        gicp_.setInputSource(current_cloud);

        Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
        if (has_ekf_) {
            Eigen::Matrix4f delta_base = prev_cloud_ekf_pose_.inverse() * current_ekf_pose;
            initial_guess = base2sonar_ * delta_base * sonar2base_;
        }

        pcl::PointCloud<pcl::PointXYZ> aligned;
        gicp_.align(aligned, initial_guess);

        if (gicp_.hasConverged()) {
            Eigen::Matrix4f result_rel = gicp_.getFinalTransformation();
            global_pose_ = global_pose_ * result_rel;

            publishOdometry(msg->header);

            auto end_time = this->now();
            auto duration = (end_time - start_time).seconds();
            
            RCLCPP_INFO(this->get_logger(), "Aligned in %.3f sec. Score: %f", 
                        duration, gicp_.getFitnessScore());
        } else {
            RCLCPP_WARN(this->get_logger(), "FastGICP did not converge for frame at timestamp: %d.%09d", 
            msg->header.stamp.sec, msg->header.stamp.nanosec);
        }

        prev_cloud_ = current_cloud;
        prev_cloud_ekf_pose_ = current_ekf_pose;
    }

    void publishOdometry(const std_msgs::msg::Header& header)
    {
        nav_msgs::msg::Odometry odom_msg;
        
        odom_msg.header.stamp = header.stamp;
        odom_msg.header.frame_id = "odom";       
        odom_msg.child_frame_id = "sonar_link"; 

        Eigen::Vector3f t = global_pose_.block<3,1>(0,3);
        Eigen::Quaternionf q(global_pose_.block<3,3>(0,0));

        odom_msg.pose.pose.position.x = t.x();
        odom_msg.pose.pose.position.y = t.y();
        odom_msg.pose.pose.position.z = t.z();

        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();

        odom_pub_->publish(odom_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ekf_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr prev_cloud_;
    
    fast_gicp::FastGICP<pcl::PointXYZ, pcl::PointXYZ> gicp_;
    
    Eigen::Matrix4f global_pose_;
    Eigen::Matrix4f latest_ekf_pose_;
    Eigen::Matrix4f prev_cloud_ekf_pose_;
    Eigen::Matrix4f sonar2base_;
    Eigen::Matrix4f base2sonar_;
    
    std::mutex ekf_mutex_;
    bool has_ekf_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GicpOdomNode>());
    rclcpp::shutdown();
    return 0;
}