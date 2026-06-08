#include <memory>
#include <string>
#include <mutex>
#include <unordered_map>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <Eigen/Geometry>

// ── Voxel key + hash ──────────────────────────────────────────────────────────
struct VoxelKey {
    int x, y, z;
    bool operator==(const VoxelKey& o) const { return x==o.x && y==o.y && z==o.z; }
};
struct VoxelKeyHash {
    size_t operator()(const VoxelKey& k) const {
        size_t h = std::hash<int>()(k.x);
        h ^= std::hash<int>()(k.y) + 0x9e3779b9 + (h<<6) + (h>>2);
        h ^= std::hash<int>()(k.z) + 0x9e3779b9 + (h<<6) + (h>>2);
        return h;
    }
};

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
        odom_warmup_           = this->declare_parameter<int>("odom_warmup", 5);
        max_pts_per_voxel_     = this->declare_parameter<int>("max_pts_per_voxel", 20);
        max_range_             = static_cast<float>(this->declare_parameter<double>("max_range", -1.0));
        max_z_                 = static_cast<float>(this->declare_parameter<double>("max_z", -1.0));

        use_sor_        = this->declare_parameter<bool>  ("use_sor",          false);
        sor_k_          = this->declare_parameter<int>   ("sor_k",            10);
        sor_std_thresh_ = static_cast<float>(this->declare_parameter<double>("sor_std_thresh", 1.0));

        use_ror_           = this->declare_parameter<bool>  ("use_ror",          false);
        ror_min_neighbors_ = this->declare_parameter<int>   ("ror_min_neighbors", 5);
        ror_radius_        = static_cast<float>(this->declare_parameter<double>("ror_radius", 0.5));

        // sonar → base_link extrinsic (translation in metres, rotation as quaternion w,x,y,z)
        double tx = this->declare_parameter<double>("base2sonar_tx",  0.38);
        double ty = this->declare_parameter<double>("base2sonar_ty",   0.08);
        double tz = this->declare_parameter<double>("base2sonar_tz",  -0.525);
        double roll = this->declare_parameter<double>("base2sonar_r",  180.0);
        double pitch = this->declare_parameter<double>("base2sonar_p",   30.0);
        double yaw = this->declare_parameter<double>("base2sonar_y",  0.000);


        Eigen::Quaternionf rotation_sb;
        rotation_sb = Eigen::AngleAxisf(static_cast<float>(yaw   * M_PI / 180.0), Eigen::Vector3f::UnitZ())
                    * Eigen::AngleAxisf(static_cast<float>(pitch  * M_PI / 180.0), Eigen::Vector3f::UnitY())
                    * Eigen::AngleAxisf(static_cast<float>(roll   * M_PI / 180.0), Eigen::Vector3f::UnitX());

        base2sonar_ = Eigen::Matrix4f::Identity();
        base2sonar_.block<3,3>(0,0) = rotation_sb.toRotationMatrix();
        base2sonar_.block<3,1>(0,3) = Eigen::Vector3f(tx, ty, tz);
        sonar2base_ = base2sonar_.inverse();

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
        RCLCPP_INFO(get_logger(), "Waiting for %d odom msgs before mapping. Max %d pts/voxel.",
            odom_warmup_, max_pts_per_voxel_);
    }

private:
    // ── Odometry callback ─────────────────────────────────────────────────────
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
        if (odom_count_ < odom_warmup_) ++odom_count_;
    }

    // ── Point cloud callback ──────────────────────────────────────────────────
    // Transform chain:  p_odom = T_odom_base * T_base_sonar * p_sonar
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
            if (odom_count_ < odom_warmup_) {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                    "Odom warmup: %d / %d msgs received, holding off mapping.",
                    odom_count_, odom_warmup_);
                return;
            }
            T_odom_base = latest_odom_pose_;
        }

        // Decode incoming cloud (sonar frame)
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sonar(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud_sonar);

        if (cloud_sonar->empty()) return;

        // Max range filter (sonar frame)
        if (max_range_ >= 0.0f) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
            filtered->reserve(cloud_sonar->size());
            for (const auto& pt : cloud_sonar->points)
                if (pt.x <= max_range_) filtered->push_back(pt);
            cloud_sonar = filtered;
        }

        // Max Z filter (sonar frame)
        if (max_z_ >= 0.0f) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
            filtered->reserve(cloud_sonar->size());
            for (const auto& pt : cloud_sonar->points)
                if (pt.z <= max_z_) filtered->push_back(pt);
            cloud_sonar = filtered;
        }

        // SOR outlier removal (sonar frame, before transform)
        if (use_sor_) {
            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
            sor.setInputCloud(cloud_sonar);
            sor.setMeanK(sor_k_);
            sor.setStddevMulThresh(sor_std_thresh_);
            sor.filter(*cloud_sonar);
        }

        // ROR outlier removal (sonar frame, before transform)
        if (use_ror_) {
            pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
            ror.setInputCloud(cloud_sonar);
            ror.setRadiusSearch(ror_radius_);
            ror.setMinNeighborsInRadius(ror_min_neighbors_);
            ror.filter(*cloud_sonar);
        }

        if (cloud_sonar->empty()) return;

        // Transform: sonar → odom
        Eigen::Matrix4f T_odom_sonar = T_odom_base * base2sonar_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_odom(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*cloud_sonar, *cloud_odom, T_odom_sonar);

        // Accumulate with per-voxel density cap
        for (const auto& pt : cloud_odom->points) {
            VoxelKey key{
                static_cast<int>(std::floor(pt.x / map_resolution_)),
                static_cast<int>(std::floor(pt.y / map_resolution_)),
                static_cast<int>(std::floor(pt.z / map_resolution_))
            };
            auto& count = voxel_counts_[key];
            if (count < max_pts_per_voxel_) {
                global_map_->push_back(pt);
                ++count;
            }
        }

        // Publish every N frames
        if (pub_count_++ % publish_every_ == 0) {
            sensor_msgs::msg::PointCloud2 map_msg;
            pcl::toROSMsg(*global_map_, map_msg);
            map_msg.header.frame_id = odom_frame_;
            map_msg.header.stamp    = msg->header.stamp;
            global_map_pub_->publish(map_msg);

            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                "Global map: %zu points  voxels: %zu", global_map_->size(), voxel_counts_.size());
        }
    }

    // ── Members ───────────────────────────────────────────────────────────────
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr        odom_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr     global_map_pub_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_;
    std::unordered_map<VoxelKey, int, VoxelKeyHash> voxel_counts_;

    std::mutex       odom_mutex_;
    Eigen::Matrix4f  latest_odom_pose_{Eigen::Matrix4f::Identity()};
    bool             has_odom_{false};
    int              odom_count_{0};

    Eigen::Matrix4f  sonar2base_{Eigen::Matrix4f::Identity()};
    Eigen::Matrix4f  base2sonar_{Eigen::Matrix4f::Identity()};

    std::string odom_frame_{"odom"};
    float       map_resolution_{0.2f};
    int         publish_every_{5};
    int         pub_count_{0};
    int         odom_warmup_{5};
    int         max_pts_per_voxel_{20};
    float       max_range_{-1.0f};
    float       max_z_{-1.0f};
    bool        use_sor_{false};
    int         sor_k_{10};
    float       sor_std_thresh_{1.0f};
    bool        use_ror_{false};
    int         ror_min_neighbors_{5};
    float       ror_radius_{0.5f};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapNode>());
    rclcpp::shutdown();
    return 0;
}
