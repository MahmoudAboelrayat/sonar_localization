#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

class PriorMapPublisher : public rclcpp::Node
{
public:
    PriorMapPublisher() : Node("prior_map_publisher")
    {
        std::string map_path   = declare_parameter<std::string>("prior_map_path",       "");
        double      resolution = declare_parameter<double>     ("prior_map_resolution",  0.2);
        std::string map_frame  = declare_parameter<std::string>("map_frame",            "unity_origin");
        std::string topic      = declare_parameter<std::string>("map_topic",            "prior_map");
        double      period_s   = declare_parameter<double>     ("publish_period",        5.0);

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        if (pcl::io::loadPCDFile<pcl::PointXYZI>(map_path, *cloud) < 0)
            throw std::runtime_error("Failed to load prior map: " + map_path);

        pcl::VoxelGrid<pcl::PointXYZI> vg;
        vg.setLeafSize(
            static_cast<float>(resolution),
            static_cast<float>(resolution),
            static_cast<float>(resolution));
        vg.setInputCloud(cloud);
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>);
        vg.filter(*filtered);

        pcl::toROSMsg(*filtered, map_msg_);
        map_msg_.header.frame_id = map_frame;
        map_msg_.header.stamp    = rclcpp::Time(0);

        // transient_local = ROS2 latched — late subscribers receive the last message
        rclcpp::QoS qos(1);
        qos.transient_local().reliable();
        pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(topic, qos);
        pub_->publish(map_msg_);

        // Republish periodically for RViz reconnects
        auto period_ms = std::chrono::milliseconds(static_cast<int>(period_s * 1000));
        timer_ = create_wall_timer(period_ms, [this]() { pub_->publish(map_msg_); });

        RCLCPP_INFO(get_logger(),
            "Prior map ready: %zu pts | frame=%s | topic=%s",
            filtered->size(), map_frame.c_str(), topic.c_str());
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr                                timer_;
    sensor_msgs::msg::PointCloud2                               map_msg_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<PriorMapPublisher>());
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("prior_map_publisher"), "%s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
