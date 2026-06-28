#include <memory>
#include <string>
#include <mutex>
#include <unordered_map>
#include <unordered_set>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <Eigen/Geometry>

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

class DenseMapNode : public rclcpp::Node
{
public:
    DenseMapNode() : Node("dense_mapping_node")
    {
        std::string pc_topic   = this->declare_parameter<std::string>("pc_topic",   "usv/point_cloud");
        std::string odom_topic = this->declare_parameter<std::string>("odom_topic", "/imu/odometry");
        std::string map_topic  = this->declare_parameter<std::string>("map_topic",  "/dense_map");
        odom_frame_            = this->declare_parameter<std::string>("odom_frame", "odom");
        save_pcd_path_         = this->declare_parameter<std::string>("save_pcd_path", "/tmp/dense_map.pcd");
        save_ply_path_         = this->declare_parameter<std::string>("save_ply_path", "/tmp/dense_mesh.ply");

        map_resolution_    = static_cast<float>(this->declare_parameter<double>("map_resolution",  0.02));
        publish_every_     = this->declare_parameter<int>("publish_every",    5);
        odom_warmup_       = this->declare_parameter<int>("odom_warmup",      5);
        max_range_         = static_cast<float>(this->declare_parameter<double>("max_range",       -1.0));
        max_z_             = static_cast<float>(this->declare_parameter<double>("max_z",           -1.0));
        save_on_shutdown_  = this->declare_parameter<bool>("save_on_shutdown", true);

        // Surface reconstruction
        reconstruct_surface_  = this->declare_parameter<bool>  ("reconstruct_surface",  true);
        surface_method_       = this->declare_parameter<std::string>("surface_method",  "gp3");  // "gp3" or "poisson"
        reconstruct_every_    = this->declare_parameter<int>   ("reconstruct_every",    50);     // every N scans
        normal_k_             = this->declare_parameter<int>   ("normal_k",             20);
        mls_radius_           = static_cast<float>(this->declare_parameter<double>("mls_radius",          0.2));
        gp3_radius_           = static_cast<float>(this->declare_parameter<double>("gp3_search_radius",   1.0));
        gp3_mu_               = this->declare_parameter<double>("gp3_mu",               3.0);
        gp3_max_nn_           = this->declare_parameter<int>   ("gp3_max_nn",           200);
        poisson_depth_        = this->declare_parameter<int>   ("poisson_depth",        8);
        min_pts_for_surface_  = this->declare_parameter<int>   ("min_pts_for_surface",  200);

        use_sor_           = this->declare_parameter<bool>  ("use_sor",          true);
        sor_k_             = this->declare_parameter<int>   ("sor_k",            10);
        sor_std_thresh_    = static_cast<float>(this->declare_parameter<double>("sor_std_thresh", 1.0));
        use_ror_           = this->declare_parameter<bool>  ("use_ror",          true);
        ror_min_neighbors_ = this->declare_parameter<int>   ("ror_min_neighbors", 5);
        ror_radius_        = static_cast<float>(this->declare_parameter<double>("ror_radius",     0.5));

        double tx    = this->declare_parameter<double>("base2sonar_tx",   0.38);
        double ty    = this->declare_parameter<double>("base2sonar_ty",   0.08);
        double tz    = this->declare_parameter<double>("base2sonar_tz",  -0.525);
        double roll  = this->declare_parameter<double>("base2sonar_r",   180.0);
        double pitch = this->declare_parameter<double>("base2sonar_p",    30.0);
        double yaw   = this->declare_parameter<double>("base2sonar_y",     0.0);

        Eigen::Quaternionf rot_sb =
            Eigen::AngleAxisf(static_cast<float>(yaw   * M_PI / 180.0), Eigen::Vector3f::UnitZ())
          * Eigen::AngleAxisf(static_cast<float>(pitch * M_PI / 180.0), Eigen::Vector3f::UnitY())
          * Eigen::AngleAxisf(static_cast<float>(roll  * M_PI / 180.0), Eigen::Vector3f::UnitX());
        base2sonar_ = Eigen::Matrix4f::Identity();
        base2sonar_.block<3,3>(0,0) = rot_sb.toRotationMatrix();
        base2sonar_.block<3,1>(0,3) = Eigen::Vector3f(tx, ty, tz);

        pc_sub_   = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            pc_topic, 20, std::bind(&DenseMapNode::pointCloudCallback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 20, std::bind(&DenseMapNode::odomCallback, this, std::placeholders::_1));

        global_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(map_topic, rclcpp::QoS(1));
        mesh_pub_       = this->create_publisher<visualization_msgs::msg::Marker>("/dense_mesh", rclcpp::QoS(1));
        global_map_     = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

        RCLCPP_INFO(get_logger(),
            "DenseMapNode ready — res=%.3f m  surface=%s (%s)  pc: %s",
            map_resolution_,
            reconstruct_surface_ ? "ON" : "OFF", surface_method_.c_str(),
            pc_topic.c_str());
    }

    ~DenseMapNode()
    {
        if (!save_on_shutdown_ || global_map_->empty()) return;

        global_map_->width = global_map_->size(); global_map_->height = 1;
        pcl::io::savePCDFileBinary(save_pcd_path_, *global_map_);
        RCLCPP_INFO(get_logger(), "Saved PCD → %s  (%zu pts)", save_pcd_path_.c_str(), global_map_->size());

        if (reconstruct_surface_ && global_map_->size() >= (size_t)min_pts_for_surface_) {
            pcl::PolygonMesh mesh;
            if (buildMesh(*global_map_, mesh)) {
                pcl::io::savePLYFile(save_ply_path_, mesh);
                RCLCPP_INFO(get_logger(), "Saved mesh → %s  (%zu polygons)",
                            save_ply_path_.c_str(), mesh.polygons.size());
            }
        }
    }

private:
    // ── Odometry ──────────────────────────────────────────────────────────────
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        Eigen::Quaternionf q(
            static_cast<float>(msg->pose.pose.orientation.w),
            static_cast<float>(msg->pose.pose.orientation.x),
            static_cast<float>(msg->pose.pose.orientation.y),
            static_cast<float>(msg->pose.pose.orientation.z));
        if (q.norm() < 1e-6f) return;

        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        pose.block<3,3>(0,0) = q.normalized().toRotationMatrix();
        pose.block<3,1>(0,3) = Eigen::Vector3f(
            static_cast<float>(msg->pose.pose.position.x),
            static_cast<float>(msg->pose.pose.position.y),
            static_cast<float>(msg->pose.pose.position.z));

        std::lock_guard<std::mutex> lk(odom_mutex_);
        latest_odom_pose_ = pose;
        has_odom_ = true;
        if (odom_count_ < odom_warmup_) ++odom_count_;
    }

    // ── Point cloud ───────────────────────────────────────────────────────────
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        Eigen::Matrix4f T_odom_base;
        {
            std::lock_guard<std::mutex> lk(odom_mutex_);
            if (!has_odom_ || odom_count_ < odom_warmup_) return;
            T_odom_base = latest_odom_pose_;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);
        if (cloud->empty()) return;

        // Range / Z filters
        if (max_range_ >= 0.0f) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
            for (const auto& p : cloud->points)
                if (std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z) <= max_range_) tmp->push_back(p);
            cloud = tmp;
        }
        if (max_z_ >= 0.0f) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
            for (const auto& p : cloud->points) if (p.z <= max_z_) tmp->push_back(p);
            cloud = tmp;
        }

        // Per-scan outlier removal
        if (use_sor_ && cloud->size() >= 3) {
            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
            sor.setInputCloud(cloud); sor.setMeanK(sor_k_); sor.setStddevMulThresh(sor_std_thresh_);
            sor.filter(*cloud);
        }
        if (use_ror_ && cloud->size() >= 2) {
            pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
            ror.setInputCloud(cloud); ror.setRadiusSearch(ror_radius_);
            ror.setMinNeighborsInRadius(ror_min_neighbors_);
            ror.filter(*cloud);
        }
        if (cloud->empty()) return;

        // Transform sonar → odom, deduplicate
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_w(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*cloud, *cloud_w, T_odom_base * base2sonar_);

        for (const auto& p : cloud_w->points) {
            VoxelKey key{
                static_cast<int>(std::floor(p.x / map_resolution_)),
                static_cast<int>(std::floor(p.y / map_resolution_)),
                static_cast<int>(std::floor(p.z / map_resolution_))
            };
            if (voxel_seen_.insert(key).second)
                global_map_->push_back(p);
        }

        // Publish point cloud
        if (pub_count_ % publish_every_ == 0) {
            sensor_msgs::msg::PointCloud2 out;
            pcl::toROSMsg(*global_map_, out);
            out.header.frame_id = odom_frame_;
            out.header.stamp    = msg->header.stamp;
            global_map_pub_->publish(out);
            last_stamp_ = msg->header.stamp;
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
                "Dense map: %zu pts", global_map_->size());
        }

        // Surface reconstruction every N scans
        if (reconstruct_surface_ &&
            (int)global_map_->size() >= min_pts_for_surface_ &&
            pub_count_ % reconstruct_every_ == 0)
        {
            pcl::PolygonMesh mesh;
            if (buildMesh(*global_map_, mesh))
                publishMesh(mesh, msg->header.stamp);
        }

        ++pub_count_;
    }

    // ── Surface reconstruction ────────────────────────────────────────────────
    bool buildMesh(const pcl::PointCloud<pcl::PointXYZ>& cloud_in, pcl::PolygonMesh& mesh_out)
    {
        // 1. MLS smoothing + normal estimation
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_n(new pcl::PointCloud<pcl::PointNormal>);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

        pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
        mls.setInputCloud(cloud_in.makeShared());
        mls.setSearchMethod(tree);
        mls.setSearchRadius(mls_radius_);
        mls.setComputeNormals(true);
        mls.setPolynomialOrder(2);
        mls.process(*cloud_n);

        if (cloud_n->empty()) {
            RCLCPP_WARN(get_logger(), "MLS produced empty cloud — skipping surface reconstruction");
            return false;
        }

        // 2. Surface reconstruction
        if (surface_method_ == "poisson") {
            pcl::Poisson<pcl::PointNormal> poisson;
            poisson.setInputCloud(cloud_n);
            poisson.setDepth(poisson_depth_);
            poisson.reconstruct(mesh_out);
        } else {
            // Greedy Projection Triangulation — faster, better for open surfaces
            pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
            tree2->setInputCloud(cloud_n);

            pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
            gp3.setInputCloud(cloud_n);
            gp3.setSearchMethod(tree2);
            gp3.setSearchRadius(gp3_radius_);
            gp3.setMu(gp3_mu_);
            gp3.setMaximumNearestNeighbors(gp3_max_nn_);
            gp3.setMaximumSurfaceAngle(M_PI / 4);  // 45°
            gp3.setMinimumAngle(M_PI / 18);         // 10°
            gp3.setMaximumAngle(2 * M_PI / 3);      // 120°
            gp3.setNormalConsistency(false);
            gp3.reconstruct(mesh_out);
        }

        RCLCPP_INFO(get_logger(), "[Surface] %s → %zu triangles from %zu pts",
                    surface_method_.c_str(), mesh_out.polygons.size(), cloud_n->size());
        return !mesh_out.polygons.empty();
    }

    // ── Publish mesh as RViz TRIANGLE_LIST marker ─────────────────────────────
    void publishMesh(const pcl::PolygonMesh& mesh, const rclcpp::Time& stamp)
    {
        // Extract vertex positions from the mesh cloud
        pcl::PointCloud<pcl::PointXYZ> verts;
        pcl::fromPCLPointCloud2(mesh.cloud, verts);

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = odom_frame_;
        marker.header.stamp    = stamp;
        marker.ns              = "dense_mesh";
        marker.id              = 0;
        marker.type            = visualization_msgs::msg::Marker::TRIANGLE_LIST;
        marker.action          = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = marker.scale.y = marker.scale.z = 1.0;
        marker.color.r = 0.2f; marker.color.g = 0.7f;
        marker.color.b = 0.9f; marker.color.a = 0.8f;
        marker.pose.orientation.w = 1.0;

        marker.points.reserve(mesh.polygons.size() * 3);
        for (const auto& poly : mesh.polygons) {
            if (poly.vertices.size() != 3) continue;
            for (int v = 0; v < 3; ++v) {
                const auto& pt = verts[poly.vertices[v]];
                geometry_msgs::msg::Point p;
                p.x = pt.x; p.y = pt.y; p.z = pt.z;
                marker.points.push_back(p);
            }
        }
        mesh_pub_->publish(marker);
    }

    // ── Members ───────────────────────────────────────────────────────────────
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr        odom_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr     global_map_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr   mesh_pub_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr               global_map_;
    std::unordered_set<VoxelKey, VoxelKeyHash>         voxel_seen_;

    std::mutex       odom_mutex_;
    Eigen::Matrix4f  latest_odom_pose_{Eigen::Matrix4f::Identity()};
    bool             has_odom_{false};
    int              odom_count_{0};
    Eigen::Matrix4f  base2sonar_{Eigen::Matrix4f::Identity()};
    rclcpp::Time     last_stamp_{0, 0, RCL_ROS_TIME};

    std::string odom_frame_{"odom"};
    std::string save_pcd_path_{"/tmp/dense_map.pcd"};
    std::string save_ply_path_{"/tmp/dense_mesh.ply"};
    float       map_resolution_{0.02f};
    int         publish_every_{5};
    int         pub_count_{0};
    int         odom_warmup_{5};
    float       max_range_{-1.0f};
    float       max_z_{-1.0f};
    bool        save_on_shutdown_{true};

    bool        reconstruct_surface_{true};
    std::string surface_method_{"gp3"};
    int         reconstruct_every_{50};
    int         normal_k_{20};
    float       mls_radius_{0.1f};
    float       gp3_radius_{1.0f};
    double      gp3_mu_{3.0};
    int         gp3_max_nn_{200};
    int         poisson_depth_{8};
    int         min_pts_for_surface_{200};

    bool  use_sor_{true};
    int   sor_k_{10};
    float sor_std_thresh_{1.0f};
    bool  use_ror_{true};
    int   ror_min_neighbors_{5};
    float ror_radius_{0.5f};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DenseMapNode>());
    rclcpp::shutdown();
    return 0;
}