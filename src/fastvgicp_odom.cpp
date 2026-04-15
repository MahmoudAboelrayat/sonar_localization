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

#include <fast_gicp/gicp/fast_vgicp.hpp>
#include <Eigen/Geometry>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

struct Keyframe {
    Eigen::Matrix4f pose;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud; 
};

class GicpOdomNode : public rclcpp::Node
{
public:
    GicpOdomNode() : Node("vgicp_odom_node")
    {
        std::string odom_pub_topic = this->declare_parameter<std::string>("topics.odom_pub", "vgicp_odom");
        std::string pc_sub_topic   = this->declare_parameter<std::string>("topics.pc_sub", "/sonar/point_cloud_noisy");
        std::string deadreckoining_sub  = this->declare_parameter<std::string>("topics.deadreckoining_sub", "/odometry/filtered");
        std::string map_pub_topic  = this->declare_parameter<std::string>("topics.map_pub", "vgicp_global_map");

        double map_res   = this->declare_parameter<double>("tuning.map_res", 0.1);
        int    vgicp_threads   = this->declare_parameter<int>("tuning.vgicp_threads", 4);
        double vgicp_epsilon   = this->declare_parameter<double>("tuning.vgicp_epsilon", 1e-4);
        double vgicp_max_dist  = this->declare_parameter<double>("tuning.vgicp_max_dist", 1.5);
        int    vgicp_max_iter  = this->declare_parameter<int>("tuning.vgicp_max_iter", 50);
        double vgicp_res       = this->declare_parameter<double>("tuning.vgicp_resolution", 0.25);
        int max_lost_frames = this->declare_parameter<int>("tuning.max_lost_frames", 40);
        
        double s2b_t_x = this->declare_parameter<double>("tf.sonar2base_x", -0.545);
        double s2b_t_y = this->declare_parameter<double>("tf.sonar2base_y", 0.000);
        double s2b_t_z = this->declare_parameter<double>("tf.sonar2base_z", -0.404);
        double s2b_roll  = this->declare_parameter<double>("tf.sonar2base_roll", 0.0);
        double s2b_pitch = this->declare_parameter<double>("tf.sonar2base_pitch", -30); // in degrees (will be converted later)
        double s2b_yaw   = this->declare_parameter<double>("tf.sonar2base_yaw", 0.0);
        
        odom_frame = this ->declare_parameter<std::string>("frames.odom_frame", "odom");
        base_frame = this ->declare_parameter<std::string>("frames.base_frame", "sam_auv_v1/base_link");


        ekf_z = this->declare_parameter<bool>("ekf_z", false);


        bool is_ned_ = this->declare_parameter<bool>("is_ned", false);
        is_ned = is_ned_;



        // KF and submap parameters
        submap_size_     = this->declare_parameter<int>   ("keyframe.submap_size",    20);
        kf_dist_thresh_  = this->declare_parameter<double>("keyframe.dist_thresh",    0.5);
        kf_angle_thresh_ = this->declare_parameter<double>("keyframe.angle_thresh",   10.0);
        
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_pub_topic, 10);
        
        pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            pc_sub_topic, 10,
            std::bind(&GicpOdomNode::pointCloudCallback, this, std::placeholders::_1)
        );

        ekf_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            deadreckoining_sub, 10,
            std::bind(&GicpOdomNode::ekfCallback, this, std::placeholders::_1)
        );

        global_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(map_pub_topic, 1);
        std::string history_map_pub_topic = this->declare_parameter<std::string>("topics.history_map_pub", "vgicp_full_map");

        history_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(history_map_pub_topic, 1);

        std::string filtered_pc_pub_topic = this->declare_parameter<std::string>("topics.filtered_pc_pub", "filtered_cloud");
        filtered_pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(filtered_pc_pub_topic, 10);

        global_pose_ = Eigen::Matrix4f::Identity();
        latest_ekf_pose_ = Eigen::Matrix4f::Identity();
        prev_ekf_pose_ = Eigen::Matrix4f::Identity();

        // Initialize Map
        local_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        map_filter_.setLeafSize(map_res, map_res, map_res); 

        // Sensor to base transformation
        // Eigen::Vector3f translation_sb(-0.545f, 0.000f, -0.404f);
        // Eigen::Quaternionf rotation_sb(0.966f, -0.000f, -0.259f, -0.000f);
        
        Eigen::Vector3f translation_sb(
            static_cast<float>(s2b_t_x), 
            static_cast<float>(s2b_t_y), 
            static_cast<float>(s2b_t_z)
        );
        
    
        Eigen::Quaternionf rotation_sb;
        rotation_sb = Eigen::AngleAxisf(static_cast<float>(s2b_yaw * M_PI / 180.0),   Eigen::Vector3f::UnitZ())
                    * Eigen::AngleAxisf(static_cast<float>(s2b_pitch * M_PI / 180.0), Eigen::Vector3f::UnitY())
                    * Eigen::AngleAxisf(static_cast<float>(s2b_roll * M_PI / 180.0),  Eigen::Vector3f::UnitX());
        
        sonar2base_ = Eigen::Matrix4f::Identity();
        sonar2base_.block<3,3>(0,0) = rotation_sb.toRotationMatrix();
        sonar2base_.block<3,1>(0,3) = translation_sb;
        
        base2sonar_ = sonar2base_.inverse();
        
        
        // FastVGICP Settings
        vgicp_.setNumThreads(vgicp_threads);               
        vgicp_.setTransformationEpsilon(vgicp_epsilon);
        vgicp_.setMaxCorrespondenceDistance(vgicp_max_dist); 
        vgicp_.setMaximumIterations(vgicp_max_iter);   
        vgicp_.setResolution(vgicp_res); 

        // convert to ned frame
        // Eigen::Quaternionf rotaionn_nd;
        // rotaionn_nd = Eigen::AngleAxisf(static_cast<float>(0.0),   Eigen::Vector3f::UnitZ())
        // * Eigen::AngleAxisf(static_cast<float>(M_PI), Eigen::Vector3f::UnitY())
        // * Eigen::AngleAxisf(static_cast<float>(0.0),  Eigen::Vector3f::UnitX());
        
        // ned_transform = Eigen::Matrix4f::Identity();
        // ned_transform.block<3,3>(0,0) = rotaionn_nd.toRotationMatrix();

        ned_transform << 1,  0,  0,  0,
           0, -1,  0,  0,
           0,  0, -1,  0,
           0,  0,  0,  1;

        

        // setup outlier removal (optional)
        filter_outliers = this->declare_parameter<bool>("outlier_removal.filter_outliers", true);
        int num_neighbors = this->declare_parameter<int>("outlier_removal.num_neighbors", 20);
        sor_.setMeanK(num_neighbors);
        double stddev_mul_thresh = this->declare_parameter<double>("outlier_removal.stddev_mul_thresh", 1.5);
        sor_.setStddevMulThresh(stddev_mul_thresh);

        // setup radius outlier removal (optional)
        filter_radius_outliers_ = this->declare_parameter<bool>("radius_outlier_removal.filter_radius_outliers", true);
        double ror_radius = this->declare_parameter<double>("radius_outlier_removal.search_radius", 0.5);
        int ror_min_neighbors = this->declare_parameter<int>("radius_outlier_removal.min_neighbors", 5);
        ror_.setRadiusSearch(ror_radius);
        ror_.setMinNeighborsInRadius(ror_min_neighbors);

        // setup intensity filter (optional)
        filter_intensity = this->declare_parameter<bool>("intensity_filter.filter_intensity", false);
        min_intensity = this->declare_parameter<double>("intensity_filter.min_intensity", 0.0);

        RCLCPP_INFO(this->get_logger(), "FastVGICP Scan-to-Map Node initialized. Waiting for data...");
    }

private:

    void updateSubmap()
    {
        local_map_->clear();
        
        int start = std::max(0, static_cast<int>(keyframes_.size()) - submap_size_);
        for (int i = start; i < static_cast<int>(keyframes_.size()); ++i) {
            pcl::PointCloud<pcl::PointXYZI> transformed;
            pcl::transformPointCloud(*keyframes_[i].cloud, transformed, keyframes_[i].pose);
            *local_map_ += transformed;
        }
        
        pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZI>);
        map_filter_.setInputCloud(local_map_);
        map_filter_.filter(*downsampled);
        local_map_ = downsampled;
        
        RCLCPP_DEBUG(get_logger(), "Submap updated: %zu keyframes, %zu pts",
        keyframes_.size(), local_map_->size());
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr buildFullMap()
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr full_map(new pcl::PointCloud<pcl::PointXYZI>);
        for (const auto& kf : keyframes_) {
            pcl::PointCloud<pcl::PointXYZI> transformed;
            pcl::transformPointCloud(*kf.cloud, transformed, kf.pose);
            *full_map += transformed;
        }
        pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZI>);
        map_filter_.setInputCloud(full_map);
        map_filter_.filter(*downsampled);
        return downsampled;
    }

    void AddKeyFrame(const Eigen::Matrix4f& current_pose, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
    {
        Eigen::Matrix4f delta = keyframes_.back().pose.inverse() * current_pose;
        float dist  = delta.block<3,1>(0,3).norm();
        float angle = Eigen::AngleAxisf(Eigen::Matrix3f(delta.block<3,3>(0,0))).angle()
                    * 180.0f / M_PI;

        bool moved_dis = dist > static_cast<float>(kf_dist_thresh_);
        bool moved_ang = angle > static_cast<float>(kf_angle_thresh_);
        if (keyframes_.empty() || moved_dis || moved_ang) {
            Keyframe kf;
            kf.pose = current_pose;
            kf.cloud = cloud;
            keyframes_.push_back(kf);
            updateSubmap();
            RCLCPP_INFO(get_logger(), "Keyframe %zu added. Submap: %zu pts",
            keyframes_.size(), local_map_->size());
        }
    }

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
        if (!has_ekf_) {
            RCLCPP_WARN_ONCE(this->get_logger(), "Waiting for first EKF message to anchor SLAM...");
            return;
        }

        auto start_time = this->now();

        if (has_intensity_ == -1) {
            has_intensity_ = 0;
            for (const auto& field : msg->fields) {
                if (field.name == "intensity") { has_intensity_ = 1; break; }
            }
            RCLCPP_INFO(get_logger(), "Point cloud has intensity field: %s", has_intensity_ ? "yes" : "no");
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        if (has_intensity_) {
            pcl::fromROSMsg(*msg, *current_cloud);
        } else {
            pcl::PointCloud<pcl::PointXYZ> xyz_cloud;
            pcl::fromROSMsg(*msg, xyz_cloud);
            current_cloud->reserve(xyz_cloud.size());
            for (const auto& pt : xyz_cloud) {
                pcl::PointXYZI p;
                p.x = pt.x; p.y = pt.y; p.z = pt.z;
                p.intensity = 1.0f;
                current_cloud->push_back(p);
            }
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr intensity_filtered(new pcl::PointCloud<pcl::PointXYZI>);
        intensity_filtered->reserve(current_cloud->size());
        if (filter_intensity){
            for (const auto& pt : *current_cloud) {
                if (std::isfinite(pt.x) && pt.intensity > min_intensity) {
                    intensity_filtered->push_back(pt);
                }
            }
            RCLCPP_DEBUG(get_logger(), "Intensity filter: %zu → %zu pts",
                current_cloud->size(), intensity_filtered->size());
        }
        else{
            intensity_filtered = current_cloud;
        }

        if (intensity_filtered->empty()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "All points removed by intensity filter (threshold=%.2f) — lower min_intensity",
                min_intensity);
            return;
        }

        // Transform cloud from sonar frame to base_link frame 
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in_base(new pcl::PointCloud<pcl::PointXYZI>);
        if(is_ned){
            pcl::transformPointCloud(*intensity_filtered, *cloud_in_base, ned_transform * base2sonar_);
        }
        else{
        pcl::transformPointCloud(*intensity_filtered, *cloud_in_base, base2sonar_);
        }
        pcl::PointCloud<pcl::PointXYZI>::Ptr ror_out(new pcl::PointCloud<pcl::PointXYZI>);
        if (filter_radius_outliers_) {
            ror_.setInputCloud(cloud_in_base);
            ror_.filter(*ror_out);
        } else {
            ror_out = cloud_in_base;
        }
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>);
        if (filter_outliers){
            sor_.setInputCloud(ror_out);
            sor_.filter(*filtered);
        }
        else{
            filtered = ror_out;
        }

        sensor_msgs::msg::PointCloud2 filtered_msg;
        pcl::toROSMsg(*filtered, filtered_msg);
        filtered_msg.header = msg->header;
        filtered_msg.header.frame_id = base_frame;
        filtered_pc_pub_->publish(filtered_msg);

        // publish filtered point cloud 
        
            
        Eigen::Matrix4f current_ekf_pose;
        {
            std::lock_guard<std::mutex> lock(ekf_mutex_);
            current_ekf_pose = latest_ekf_pose_ ;
        }
        
        if (local_map_->empty()) {
            RCLCPP_INFO(this->get_logger(), "is ned frame: %d", is_ned);
            global_pose_ = current_ekf_pose;
            prev_ekf_pose_ = current_ekf_pose;
            
            // pcl::transformPointCloud(*current_cloud, *local_map_, global_pose_);
           
            // pcl::transformPointCloud(*cloud_in_base, *local_map_, global_pose_);
            // pcl::transformPointCloud(*filtered, *local_map_, global_pose_);
            Keyframe kf0;
            kf0.pose  = global_pose_;
            kf0.cloud = filtered;
            keyframes_.push_back(kf0);
            updateSubmap();
            RCLCPP_INFO(get_logger(), "Map initialized. First keyframe added.");
            return;
            
        }

        
        Eigen::Matrix4f ekf_delta = prev_ekf_pose_.inverse() * current_ekf_pose;
        
        Eigen::Matrix4f initial_guess = global_pose_ * ekf_delta;

        vgicp_.setInputTarget(local_map_);     
        // vgicp_.setInputSource(current_cloud);  
        // vgicp_.setInputSource(cloud_in_base);  
        vgicp_.setInputSource(filtered);  



        pcl::PointCloud<pcl::PointXYZI> aligned;
        vgicp_.align(aligned, initial_guess);

        if (vgicp_.hasConverged()) {
            lost_frames = 0;
            
            global_pose_ = vgicp_.getFinalTransformation();
            if (ekf_z){
                global_pose_(2,3) = current_ekf_pose(2,3); // Keep EKF's Z
            }
            // global_pose_ = current_ekf_pose;
            publishOdometry(msg->header);

            auto duration = (this->now() - start_time).seconds();
            RCLCPP_INFO(this->get_logger(), "Scan-to-Map Aligned. Time: %.3fs | Score: %f | Map Size: %zu", 
                      duration, vgicp_.getFitnessScore(), local_map_->size());
            RCLCPP_INFO(this->get_logger(), "Number of reseting pose=%d", 
                     reset_counter);

            // --- UPDATE THE MAP ---
            // pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan(new pcl::PointCloud<pcl::PointXYZI>);
            // pcl::transformPointCloud(*filtered, *transformed_scan, global_pose_); // Transform to global frame using the optimized pose
            // // pcl::transformPointCloud(*current_cloud, *transformed_scan, global_pose_); // Transform to global frame using the optimized pose
            
            // // Add to the local map
            // *local_map_ += *transformed_scan;

            // pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_map(new pcl::PointCloud<pcl::PointXYZI>);
            // map_filter_.setInputCloud(local_map_);
            // map_filter_.filter(*filtered_map);
            // local_map_ = filtered_map; // Swap the pointers

            // Add key frame and update the submap
            AddKeyFrame(global_pose_, filtered);

            // Publish the map for RViz every 5 frames
            static int map_pub_count = 0;
            static int history_pub_count = 0;
            if (map_pub_count++ % 5 == 0) {
                sensor_msgs::msg::PointCloud2 map_msg;
                pcl::toROSMsg(*local_map_, map_msg);
                map_msg.header.frame_id = odom_frame;
                map_msg.header.stamp = msg->header.stamp;
                global_map_pub_->publish(map_msg);
            }
            // Publish full history map (all keyframes) every 20 frames
            if (history_pub_count++ % 20 == 0) {
                auto full_map = buildFullMap();
                sensor_msgs::msg::PointCloud2 history_msg;
                pcl::toROSMsg(*full_map, history_msg);
                history_msg.header.frame_id = odom_frame;
                history_msg.header.stamp = msg->header.stamp;
                history_map_pub_->publish(history_msg);
            }

            prev_ekf_pose_ = current_ekf_pose;

        } else {
            if(lost_frames < max_lost_frames){
                RCLCPP_WARN(this->get_logger(), "VGICP did not converge! Trusting EKF for this step.");
                lost_frames++;

                // Fall back to the EKF guess
                global_pose_ = initial_guess; 
                prev_ekf_pose_ = current_ekf_pose;
            }
            else{
                RCLCPP_ERROR(this->get_logger(), "Lost track! Clearing map and resetting.");
                reset_counter++;
                lost_frames = 0;
                local_map_->clear(); 
                vgicp_.clearSource();
                vgicp_.clearTarget();
            }
        }
    }

    void publishOdometry(const std_msgs::msg::Header& header)
    {
        nav_msgs::msg::Odometry odom_msg;
        
        odom_msg.header.stamp = header.stamp;
        odom_msg.header.frame_id = odom_frame;       
        odom_msg.child_frame_id = base_frame; 
        
        // Convert the Sonar global pose back to the Robot Base global pose
        // base_odom = global_pose_ * base2sonar_; 
        base_odom = global_pose_;
        
        Eigen::Vector3f t = base_odom.block<3,1>(0,3);
        Eigen::Quaternionf q(base_odom.block<3,3>(0,0));

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
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr history_map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pc_pub_;

    fast_gicp::FastVGICP<pcl::PointXYZI, pcl::PointXYZI> vgicp_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr local_map_;
    pcl::VoxelGrid<pcl::PointXYZI> map_filter_;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor_;
    pcl::RadiusOutlierRemoval<pcl::PointXYZI> ror_;
    double min_intensity;
    bool filter_intensity;
    
    Eigen::Matrix4f global_pose_;
    Eigen::Matrix4f latest_ekf_pose_;
    Eigen::Matrix4f prev_ekf_pose_;
    Eigen::Matrix4f sonar2base_;
    Eigen::Matrix4f base2sonar_;
    Eigen::Matrix4f base_odom;

    Eigen::Matrix4f ned_transform;
    bool is_ned;
    bool ekf_z;
    bool filter_outliers;
    bool filter_radius_outliers_;
    std::string odom_frame;
    std::string base_frame;

    float odom_cov= 5e-2;

    std::mutex ekf_mutex_;
    bool has_ekf_ = false;
    int has_intensity_ = -1;
    int lost_frames = 0;
    int reset_counter = 0;
    int max_lost_frames = 40; // Increased tolerance before wiping the map


    // ----- keyframes ------
    std::vector<Keyframe> keyframes_;
    int   submap_size_      = 20;    
    double kf_dist_thresh_  = 0.5; 
    double kf_angle_thresh_ = 10.0;  // degrees
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GicpOdomNode>());
    rclcpp::shutdown();
    return 0;
}