#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/NavState.h>

class PureImuIntegrator : public rclcpp::Node
{
public:
    PureImuIntegrator() : Node("pure_imu_integrator")
    {
        // 1. Setup Publishers and Subscribers
        rclcpp::QoS qos_imu(10);

        qos_imu.reliability(rclcpp::ReliabilityPolicy::BestEffort); 
        qos_imu.durability(rclcpp::DurabilityPolicy::Volatile);
        qos_imu.history(rclcpp::HistoryPolicy::KeepLast);
        subImu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu/data", qos_imu, std::bind(&PureImuIntegrator::imuCallback, this, std::placeholders::_1));
        
        pubOdom_ = this->create_publisher<nav_msgs::msg::Odometry>("/odometry/imu", 100);
        // 2. Setup GTSAM IMU Parameters
        // (You should ideally load these noise values from a YAML parameter file)
        if(ned) {
            imuGravity = 9.81;}
        else {
            imuGravity = -9.81;
        }
        auto p = gtsam::PreintegrationParams::MakeSharedU(imuGravity);
        p->accelerometerCovariance  = gtsam::Matrix33::Identity(3,3) * pow(0.01, 2); 
        p->gyroscopeCovariance      = gtsam::Matrix33::Identity(3,3) * pow(0.001, 2);
        p->integrationCovariance    = gtsam::Matrix33::Identity(3,3) * pow(1e-4, 2);

        // 3. Initialize the Robot's Starting State
        // We assume the robot starts perfectly still at the origin (0,0,0)
        prevState_ = gtsam::NavState(gtsam::Pose3(gtsam::Rot3::Identity(), gtsam::Point3(0, 0, 0)), 
                                     gtsam::Vector3(0, 0, 0));
        
        // We assume zero initial bias (this is where drift will come from in real life)
        prevBias_ = gtsam::imuBias::ConstantBias();

        // Initialize the GTSAM integrator
        imuIntegrator_ = std::make_shared<gtsam::PreintegratedImuMeasurements>(p, prevBias_);

        RCLCPP_INFO(this->get_logger(), "Pure IMU Integrator Started!");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdom_;
    bool ned = true;
    double imuGravity;

    std::shared_ptr<gtsam::PreintegratedImuMeasurements> imuIntegrator_;
    gtsam::NavState prevState_;
    gtsam::imuBias::ConstantBias prevBias_;
    
    double lastImuTime_ = -1.0;

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        double currentImuTime = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

        // Skip the very first message just to establish a baseline time
        if (lastImuTime_ < 0) {
            lastImuTime_ = currentImuTime;
            RCLCPP_INFO(this->get_logger(), "Received first IMU message, initializing timestamp.");
            return;
        }

        double dt = currentImuTime - lastImuTime_;
        RCLCPP_INFO(this->get_logger(), "IMU dt: %f seconds", dt);
        if (dt <= 0.0) {
            RCLCPP_WARN(this->get_logger(), "Dropped IMU message! dt was %f", dt);
            return; 
        }
        lastImuTime_ = currentImuTime;

        // 1. Feed the new measurement into the integrator
        imuIntegrator_->integrateMeasurement(
            gtsam::Vector3(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z),
            gtsam::Vector3(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z), 
            dt
        );

        // 2. Predict the new state based on the previous state
        gtsam::NavState currentState = imuIntegrator_->predict(prevState_, prevBias_);

        // 3. Update our "previous" state and reset the integrator 
        // We do this step-by-step so the preintegration matrices don't grow to infinity
        prevState_ = currentState;
        imuIntegrator_->resetIntegrationAndSetBias(prevBias_);

        // 4. Publish the result as a standard ROS odometry message
        publishOdometry(currentState, msg->header.stamp);
    }

    void publishOdometry(const gtsam::NavState& state, const builtin_interfaces::msg::Time& stamp)
    {
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = "Beckholmen";
        odom.child_frame_id = "base_link";

        // Position
        odom.pose.pose.position.x = state.position().x();
        odom.pose.pose.position.y = state.position().y();
        odom.pose.pose.position.z = state.position().z();

        // Orientation
        odom.pose.pose.orientation.x = state.quaternion().x();
        odom.pose.pose.orientation.y = state.quaternion().y();
        odom.pose.pose.orientation.z = state.quaternion().z();
        odom.pose.pose.orientation.w = state.quaternion().w();

        // Velocity
        odom.twist.twist.linear.x = state.velocity().x();
        odom.twist.twist.linear.y = state.velocity().y();
        odom.twist.twist.linear.z = state.velocity().z();

        pubOdom_->publish(odom);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PureImuIntegrator>());
    rclcpp::shutdown();
    return 0;
}