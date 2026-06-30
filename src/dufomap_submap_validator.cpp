#include <deque>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/common/transforms.h>

#include <Eigen/Geometry>

#include "sonar_localization/dufomap_submap_filter.hpp"

struct Keyframe {
    Eigen::Matrix4f pose;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
};

class DufomapSubmapValidatorNode : public rclcpp::Node
{
public:
    DufomapSubmapValidatorNode(const rclcpp::NodeOptions& options)
        : Node("dufomap_submap_validator", options),
          tf_buffer_(get_clock()),
          tf_listener_(tf_buffer_)
    {
        const std::string pc_topic = declare_parameter<std::string>("topics.pc_sub", "/sonar/point_cloud");
        const std::string raw_topic = declare_parameter<std::string>("topics.raw_submap_pub", "dufomap_validator/raw_submap");
        const std::string static_topic = declare_parameter<std::string>("topics.static_submap_pub", "dufomap_validator/static_submap");
        const std::string dynamic_topic = declare_parameter<std::string>("topics.dynamic_cloud_pub", "dufomap_validator/dynamic_cloud");
        const std::string keyframe_topic = declare_parameter<std::string>("topics.keyframe_pub", "dufomap_validator/keyframe");

        odom_frame_ = declare_parameter<std::string>("frames.odom_frame", "odom");
        base_frame_ = declare_parameter<std::string>("frames.base_frame", "base_link");
        sonar_frame_ = declare_parameter<std::string>("frames.sonar_frame", "");
        use_tf_pose_ = declare_parameter<bool>("use_tf_pose", true);
        tf_lookup_timeout_ = declare_parameter<double>("tf_lookup_timeout", 0.05);
        odom_fallback_topic_ = declare_parameter<std::string>(
            "topics.odom_fallback", "");
        odom_buffer_size_ = declare_parameter<int>("odom_buffer_size", 200);

        submap_size_ = declare_parameter<int>("keyframe.submap_size", 30);
        kf_dist_thresh_ = declare_parameter<double>("keyframe.dist_thresh", 1.0);
        kf_angle_thresh_ = declare_parameter<double>("keyframe.angle_thresh", 8.0);
        map_res_ = static_cast<float>(declare_parameter<double>("tuning.map_res", 0.2));
        voxelize_output_ = declare_parameter<bool>("tuning.voxelize_output", false);
        is_ned_ = declare_parameter<bool>("is_ned", false);

        dufomap_cfg_.resolution = declare_parameter<double>("dufomap.resolution", map_res_);
        dufomap_cfg_.d_s = declare_parameter<double>("dufomap.d_s", 0.1);
        dufomap_cfg_.d_p = static_cast<std::size_t>(declare_parameter<int>("dufomap.d_p", 2));
        dufomap_cfg_.num_threads = static_cast<std::size_t>(declare_parameter<int>("dufomap.num_threads", 4));
        dufomap_cfg_.hit_extension = declare_parameter<bool>("dufomap.hit_extension", true);
        dufomap_cfg_.ray_passthrough_hits = declare_parameter<bool>("dufomap.ray_passthrough_hits", false);
        dufomap_cfg_.use_cluster = declare_parameter<bool>("dufomap.use_cluster", false);
        dufomap_cfg_.propagate = declare_parameter<bool>("dufomap.propagate", true);

        dufomap_filter_ = std::make_unique<dufomap_submap_filter::SubmapFilter>(dufomap_cfg_);

        const double b2s_x = declare_parameter<double>("tf.base2sonar_x", -0.545);
        const double b2s_y = declare_parameter<double>("tf.base2sonar_y", 0.0);
        const double b2s_z = declare_parameter<double>("tf.base2sonar_z", -0.404);
        const double b2s_roll = declare_parameter<double>("tf.base2sonar_roll", 0.0);
        const double b2s_pitch = declare_parameter<double>("tf.base2sonar_pitch", -30.0);
        const double b2s_yaw = declare_parameter<double>("tf.base2sonar_yaw", 0.0);

        use_tf_extrinsic_ = declare_parameter<bool>("use_tf_extrinsic", false);

        base2sonar_param_ = makeBase2Sonar(
            b2s_x, b2s_y, b2s_z, b2s_roll, b2s_pitch, b2s_yaw);

        ned_transform_ << 0, -1, 0, 0,
                          1, 0, 0, 0,
                          0, 0, -1, 0,
                          0, 0, 0, 1;

        filter_sor_ = declare_parameter<bool>("outlier_removal.filter_outliers", true);
        filter_ror_ = declare_parameter<bool>("radius_outlier_removal.filter_radius_outliers", true);
        sor_.setMeanK(declare_parameter<int>("outlier_removal.num_neighbors", 30));
        sor_.setStddevMulThresh(declare_parameter<double>("outlier_removal.stddev_mul_thresh", 0.7));
        ror_.setRadiusSearch(declare_parameter<double>("radius_outlier_removal.search_radius", 0.2));
        ror_.setMinNeighborsInRadius(declare_parameter<int>("radius_outlier_removal.min_neighbors_in_radius", 30));

        map_filter_.setLeafSize(map_res_, map_res_, map_res_);

        raw_submap_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(raw_topic, 1);
        static_submap_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(static_topic, 1);
        dynamic_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(dynamic_topic, 1);
        keyframe_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(keyframe_topic, 1);

        pc_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            pc_topic, 10,
            std::bind(&DufomapSubmapValidatorNode::pointCloudCallback, this, std::placeholders::_1));

        if (!odom_fallback_topic_.empty()) {
            odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
                odom_fallback_topic_, 50,
                std::bind(&DufomapSubmapValidatorNode::odomCallback, this, std::placeholders::_1));
        }

        RCLCPP_INFO(get_logger(),
            "dufomap_submap_validator ready — pc: %s  pose: %s (%s -> %s)  submap_size: %d",
            pc_topic.c_str(),
            use_tf_pose_ ? "tf" : "odom",
            odom_frame_.c_str(), base_frame_.c_str(), submap_size_);
        if (!odom_fallback_topic_.empty()) {
            RCLCPP_INFO(get_logger(),
                "Odom fallback buffer on %s (used when TF lookup fails)",
                odom_fallback_topic_.c_str());
        } else if (use_tf_pose_) {
            RCLCPP_WARN(get_logger(),
                "No odom fallback topic — TF must be available on %s -> %s",
                odom_frame_.c_str(), base_frame_.c_str());
        }
        if (use_tf_extrinsic_) {
            RCLCPP_INFO(get_logger(),
                "Using TF extrinsic %s -> %s",
                base_frame_.c_str(), sonar_frame_.c_str());
        }
    }

private:
    static Eigen::Matrix4f odomToMatrix(const nav_msgs::msg::Odometry& msg)
    {
        Eigen::Quaternionf q(
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z);
        Eigen::Vector3f t(
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z);

        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        pose.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
        pose.block<3, 1>(0, 3) = t;
        return pose;
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lk(odom_mutex_);
        odom_buffer_.push_back({rclcpp::Time(msg->header.stamp), odomToMatrix(*msg)});
        while (static_cast<int>(odom_buffer_.size()) > odom_buffer_size_) {
            odom_buffer_.pop_front();
        }
    }

    bool interpolateOdomPose(const rclcpp::Time& stamp, Eigen::Matrix4f& pose_out)
    {
        std::lock_guard<std::mutex> lk(odom_mutex_);
        if (odom_buffer_.empty()) {
            return false;
        }

        if (stamp <= odom_buffer_.front().first) {
            pose_out = odom_buffer_.front().second;
            return true;
        }
        if (stamp >= odom_buffer_.back().first) {
            pose_out = odom_buffer_.back().second;
            return true;
        }

        for (std::size_t i = 1; i < odom_buffer_.size(); ++i) {
            const auto& t1 = odom_buffer_[i - 1];
            const auto& t2 = odom_buffer_[i];
            if (stamp >= t1.first && stamp <= t2.first) {
                const double dt = (t2.first - t1.first).seconds();
                const double alpha = dt > 0.0
                    ? (stamp - t1.first).seconds() / dt
                    : 0.0;

                Eigen::Quaternionf q1(t1.second.block<3, 3>(0, 0));
                Eigen::Quaternionf q2(t2.second.block<3, 3>(0, 0));
                q1.normalize();
                q2.normalize();

                pose_out = Eigen::Matrix4f::Identity();
                pose_out.block<3, 3>(0, 0) = q1.slerp(static_cast<float>(alpha), q2).toRotationMatrix();
                pose_out.block<3, 1>(0, 3) =
                    (1.0f - static_cast<float>(alpha)) * t1.second.block<3, 1>(0, 3) +
                    static_cast<float>(alpha) * t2.second.block<3, 1>(0, 3);
                return true;
            }
        }
        return false;
    }

    bool lookupBasePose(const builtin_interfaces::msg::Time& stamp, Eigen::Matrix4f& pose_out)
    {
        const rclcpp::Time query_time(stamp);

        if (use_tf_pose_) {
            try {
                const geometry_msgs::msg::TransformStamped tf =
                    tf_buffer_.lookupTransform(
                        odom_frame_,
                        base_frame_,
                        query_time,
                        rclcpp::Duration::from_seconds(tf_lookup_timeout_));
                pose_out = tf2::transformToEigen(tf.transform).matrix().cast<float>();
                return true;
            } catch (const tf2::TransformException& ex) {
                if (odom_fallback_topic_.empty()) {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                        "TF lookup %s -> %s failed: %s",
                        odom_frame_.c_str(), base_frame_.c_str(), ex.what());
                    return false;
                }
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "TF lookup failed (%s), using odom fallback: %s",
                    ex.what(), odom_fallback_topic_.c_str());
            }
        }

        if (odom_fallback_topic_.empty()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "Waiting for odom on fallback topic");
            return false;
        }

        if (!interpolateOdomPose(query_time, pose_out)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "No odom messages buffered yet on %s", odom_fallback_topic_.c_str());
            return false;
        }
        return true;
    }

    static Eigen::Matrix4f makeBase2Sonar(
        double x, double y, double z, double roll, double pitch, double yaw)
    {
        Eigen::Quaternionf rotation_sb;
        rotation_sb = Eigen::AngleAxisf(static_cast<float>(yaw * M_PI / 180.0), Eigen::Vector3f::UnitZ())
                    * Eigen::AngleAxisf(static_cast<float>(pitch * M_PI / 180.0), Eigen::Vector3f::UnitY())
                    * Eigen::AngleAxisf(static_cast<float>(roll * M_PI / 180.0), Eigen::Vector3f::UnitX());

        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        transform.block<3, 3>(0, 0) = rotation_sb.toRotationMatrix();
        transform.block<3, 1>(0, 3) = Eigen::Vector3f(
            static_cast<float>(x),
            static_cast<float>(y),
            static_cast<float>(z));
        return transform;
    }

    bool lookupBase2Sonar(Eigen::Matrix4f& transform_out)
    {
        if (sonar_frame_.empty()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "use_tf_extrinsic is true but frames.sonar_frame is empty");
            return false;
        }
        try {
            const geometry_msgs::msg::TransformStamped tf =
                tf_buffer_.lookupTransform(
                    base_frame_,
                    sonar_frame_,
                    tf2::TimePointZero);
            transform_out = tf2::transformToEigen(tf.transform).matrix().cast<float>();
            return true;
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "TF lookup %s -> %s failed: %s",
                base_frame_.c_str(), sonar_frame_.c_str(), ex.what());
            return false;
        }
    }

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (!received_pc_) {
            received_pc_ = true;
            RCLCPP_INFO(get_logger(), "Received first point cloud on %s", pc_sub_->get_topic_name());
        }

        Eigen::Matrix4f pose;
        if (!lookupBasePose(msg->header.stamp, pose)) {
            return;
        }

        {
            std::lock_guard<std::mutex> lk(kf_mutex_);
            if (!keyframes_.empty()) {
                const Eigen::Matrix4f delta = keyframes_.back().pose.inverse() * pose;
                const float dist = delta.block<3, 1>(0, 3).norm();
                const float angle = Eigen::AngleAxisf(
                    Eigen::Matrix3f(delta.block<3, 3>(0, 0))).angle() * 180.0f / static_cast<float>(M_PI);
                if (dist < static_cast<float>(kf_dist_thresh_) &&
                    angle < static_cast<float>(kf_angle_thresh_)) {
                    return;
                }
            }
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr raw(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *raw);

        Eigen::Matrix4f base2sonar = base2sonar_param_;
        if (use_tf_extrinsic_) {
            if (!lookupBase2Sonar(base2sonar)) {
                return;
            }
        }
        const Eigen::Matrix4f sonar_to_base =
            is_ned_ ? (ned_transform_ * base2sonar) : base2sonar;
        const Eigen::Matrix4f sonar_pose = pose * sonar_to_base;

        // Filter in sensor frame — dufomap expects sensor-frame clouds and sensor pose.
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered = raw;
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
        if (filtered->empty()) {
            return;
        }

        {
            std::lock_guard<std::mutex> lk(kf_mutex_);
            Keyframe kf;
            kf.pose = sonar_pose;
            kf.cloud = filtered;
            keyframes_.push_back(kf);

            const int max_kf = submap_size_;
            if (static_cast<int>(keyframes_.size()) > max_kf) {
                keyframes_.erase(keyframes_.begin());
            }

            publishKeyframe(keyframes_.back(), msg->header.stamp);
            publishSubmaps(msg->header.stamp);
        }
    }

    void publishKeyframe(const Keyframe& kf, const builtin_interfaces::msg::Time& stamp)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud_world;
        pcl::transformPointCloud(*kf.cloud, cloud_world, kf.pose);
        publishCloud(keyframe_pub_, cloud_world, stamp);
    }

    void updateDufomapWindow(
        const std::vector<dufomap_submap_filter::KeyframeView>& views,
        int window_start)
    {
        const std::size_t kf_count = keyframes_.size();
        const bool rebuild =
            last_integrated_start_ != window_start ||
            kf_count < last_integrated_count_;

        if (rebuild) {
            dufomap_filter_->reset();
            for (const auto& view : views) {
                dufomap_filter_->integrateKeyframe(view);
            }
        } else if (kf_count > last_integrated_count_ && !views.empty()) {
            dufomap_filter_->integrateKeyframe(views.back());
        }

        last_integrated_start_ = window_start;
        last_integrated_count_ = kf_count;
    }

    void publishSubmaps(const builtin_interfaces::msg::Time& stamp)
    {
        if (keyframes_.empty()) {
            return;
        }

        const int start = std::max(0, static_cast<int>(keyframes_.size()) - submap_size_);
        const int end = static_cast<int>(keyframes_.size()) - 1;

        std::vector<dufomap_submap_filter::KeyframeView> views;
        views.reserve(static_cast<std::size_t>(end - start + 1));
        for (int i = start; i <= end; ++i) {
            dufomap_submap_filter::KeyframeView view;
            view.pose = keyframes_[static_cast<std::size_t>(i)].pose;
            view.cloud = keyframes_[static_cast<std::size_t>(i)].cloud;
            views.push_back(view);
        }

        updateDufomapWindow(views, start);

        pcl::PointCloud<pcl::PointXYZ> raw_map =
            dufomap_submap_filter::buildRawSubmap(views, 0, static_cast<int>(views.size()) - 1);
        const auto filtered = dufomap_filter_->segmentWindow(
            views, 0, static_cast<int>(views.size()) - 1);

        const std::size_t raw_n = raw_map.size();
        const std::size_t split_n = filtered.raw_point_count;
        const bool partition_ok = raw_n == split_n;

        pcl::PointCloud<pcl::PointXYZ> raw_out = raw_map;
        pcl::PointCloud<pcl::PointXYZ> static_out = filtered.static_map;
        pcl::PointCloud<pcl::PointXYZ> dynamic_out = filtered.dynamic_cloud;

        if (voxelize_output_) {
            pcl::PointCloud<pcl::PointXYZ> tmp;
            map_filter_.setInputCloud(raw_out.makeShared());
            map_filter_.filter(tmp);
            raw_out = tmp;
            map_filter_.setInputCloud(static_out.makeShared());
            map_filter_.filter(tmp);
            static_out = tmp;
            map_filter_.setInputCloud(dynamic_out.makeShared());
            map_filter_.filter(tmp);
            dynamic_out = tmp;
        }

        publishCloud(raw_submap_pub_, raw_out, stamp);
        publishCloud(static_submap_pub_, static_out, stamp);
        publishCloud(dynamic_cloud_pub_, dynamic_out, stamp);

        RCLCPP_INFO(get_logger(),
            "Published submaps — raw: %zu  static: %zu  dynamic: %zu  "
            "static+dynamic: %zu  partition_ok: %s  keyframes: %zu  "
            "diag_seen_free: %zu  diag_label_valid: %zu%s",
            raw_n, filtered.static_map.size(), filtered.dynamic_cloud.size(),
            split_n, partition_ok ? "yes" : "NO",
            keyframes_.size(),
            filtered.diag_seen_free, filtered.diag_label_valid,
            voxelize_output_ ? "  (voxelized for display)" : "");
        if (filtered.dynamic_cloud.empty() && filtered.diag_seen_free == 0) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "No hit voxels were seenFree — dufomap sees no temporal conflict. "
                "Expected for a fully static scene; check sensor pose if you expect dynamics.");
        }
        if (!partition_ok) {
            RCLCPP_WARN(get_logger(),
                "Point partition mismatch: raw=%zu vs static+dynamic=%zu",
                raw_n, split_n);
        }
    }

    void publishCloud(
        const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub,
        const pcl::PointCloud<pcl::PointXYZ>& cloud,
        const builtin_interfaces::msg::Time& stamp)
    {
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(cloud, msg);
        msg.header.stamp = stamp;
        msg.header.frame_id = odom_frame_;
        pub->publish(msg);
    }

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr raw_submap_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr static_submap_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr dynamic_cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr keyframe_pub_;

    std::unique_ptr<dufomap_submap_filter::SubmapFilter> dufomap_filter_;
    int last_integrated_start_{-1};
    std::size_t last_integrated_count_{0};

    std::mutex kf_mutex_;
    std::mutex odom_mutex_;
    std::deque<std::pair<rclcpp::Time, Eigen::Matrix4f>> odom_buffer_;
    std::vector<Keyframe> keyframes_;
    Eigen::Matrix4f base2sonar_param_{Eigen::Matrix4f::Identity()};
    Eigen::Matrix4f ned_transform_{Eigen::Matrix4f::Identity()};

    pcl::VoxelGrid<pcl::PointXYZ> map_filter_;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor_;
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror_;

    std::string odom_frame_;
    std::string base_frame_;
    std::string sonar_frame_;
    std::string odom_fallback_topic_;
    int submap_size_{30};
    int odom_buffer_size_{200};
    double kf_dist_thresh_{1.0};
    double kf_angle_thresh_{8.0};
    double tf_lookup_timeout_{0.05};
    float map_res_{0.2f};
    bool voxelize_output_{false};
    bool is_ned_{false};
    bool use_tf_pose_{true};
    bool use_tf_extrinsic_{false};
    bool received_pc_{false};
    bool filter_sor_{true};
    bool filter_ror_{true};
    dufomap_submap_filter::Config dufomap_cfg_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DufomapSubmapValidatorNode>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}
