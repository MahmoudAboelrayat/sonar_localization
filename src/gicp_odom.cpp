#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>

#include <Eigen/Geometry>
#include <std_msgs/msg/header.hpp>
#include <pcl/filters/voxel_grid.h>

class SonarOdometryNode : public rclcpp::Node
{
public:
    SonarOdometryNode() : Node("sonar_odometry_node")
    {
        // Publisher and Subscriber
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/sonar/point_cloud", 10,
            std::bind(&SonarOdometryNode::pointCloudCallback, this, std::placeholders::_1)
        );

        // Initialize Global Pose
        global_pose_ = Eigen::Matrix4f::Identity();

        // Configure GICP
        gicp_.setTransformationEpsilon(1e-8);
        gicp_.setMaxCorrespondenceDistance(0.2);
        gicp_.setMaximumIterations(50);
        gicp_.setCorrespondenceRandomness(20);

        RCLCPP_INFO(this->get_logger(), "Sonar Odometry Node initialized. Waiting for point clouds...");
    }


private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampleCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(input_cloud);
        voxel_filter.setLeafSize(0.2f, 0.2f, 0.2f); 
        voxel_filter.filter(*filtered_cloud);
        
        return filtered_cloud;
    }
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // 1. Convert ROS PointCloud2 to PCL PointCloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *current_cloud);

        // skip first frame
        if (!prev_cloud_) {
            prev_cloud_ = current_cloud;
            return;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_perv = downsampleCloud(prev_cloud_);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_current = downsampleCloud(current_cloud);

        gicp_.setInputTarget(filtered_perv);
        gicp_.setInputSource(filtered_current);

        pcl::PointCloud<pcl::PointXYZ> aligned;
        gicp_.align(aligned);

        if (gicp_.hasConverged()) {
            Eigen::Matrix4f result_rel = gicp_.getFinalTransformation();
            global_pose_ = global_pose_ * result_rel;

            publishOdometry(msg->header);

            RCLCPP_INFO(this->get_logger(), "Frame processed. Score: %f", gicp_.getFitnessScore());
        } else {
            RCLCPP_WARN(this->get_logger(), "GICP did not converge for this frame!");
        }

        prev_cloud_ = current_cloud;
    }

    void publishOdometry(const std_msgs::msg::Header& header)    {
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
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr prev_cloud_;
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp_;
    Eigen::Matrix4f global_pose_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SonarOdometryNode>());
    rclcpp::shutdown();
    return 0;
}