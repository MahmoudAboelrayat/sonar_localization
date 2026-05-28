#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from interfaces.msg import IMU, AHRS
from sensor_msgs.msg import Imu
import numpy as np

class ImuBridge(Node):
    def __init__(self):
        super().__init__("imu_bridge")
        self.get_logger().info("IMU bridge started")

        self.declare_parameter('frame_id', 'dvl_link')
        self.declare_parameter('has_ahrs', False)
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.has_ahrs = self.get_parameter('has_ahrs').get_parameter_value().bool_value

        self.declare_parameter('input_topic', '/nucleus_node/imu_packets')
        self.declare_parameter('ahrs_topic', '/nucleus_node/ahrs_packets')
        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.ahrs_topic = self.get_parameter('ahrs_topic').get_parameter_value().string_value
        self.declare_parameter('output_topic', '/dvl/imu')
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value


        if self.has_ahrs:
            self.ahrs_Sub = self.create_subscription(AHRS, self.ahrs_topic, self.ahrs_callback, 10)

        self.sub = self.create_subscription(IMU, self.input_topic, self.imu_callback, 10)
        # self.ahrs_sub = self.create_subscription(AHRS, '/nucleus_node/rs_packets',self.ahrs_callback,10)
        self.pub = self.create_publisher(Imu, self.output_topic, 10)
        self.linear_accel_noise = 9.4e-8  # m/s^2
        self.angular_vel_noise = 5.7e-8   # rad/s
        self.orientation_noise_deg = 5e-8 # Degrees of jitter
        self.orientation_noise_rad = np.radians(self.orientation_noise_deg)


        self.declare_parameter('sync_max_age', 0.02)  # seconds
        self.sync_max_age = self.get_parameter('sync_max_age').get_parameter_value().double_value

        self.q_w = 1.0
        self.q_x = 0.0
        self.q_y = 0.0
        self.q_z = 0.0
        self.ahrs_stamp = None  # timestamp of latest AHRS message

    def ahrs_callback(self, msg):
        self.q_x = msg.quaternion_x
        self.q_y = msg.quaternion_y
        self.q_z = msg.quaternion_z
        self.q_w = msg.quaternion_w
        self.ahrs_stamp = self.get_clock().now()

    def imu_callback(self, msg):
        if not msg.is_valid:
            self.get_logger().warn("Invalid IMU packet, skipping.")
            return

        # Sync check: reject if AHRS data has never arrived or is too old
        if self.has_ahrs:
            if self.ahrs_stamp is None:
                self.get_logger().warn_once("Waiting for first AHRS message...")
                return
            ahrs_age = (self.get_clock().now() - self.ahrs_stamp).nanoseconds * 1e-9
            if ahrs_age > self.sync_max_age:
                self.get_logger().warn(
                    f"AHRS too old ({ahrs_age:.3f}s > {self.sync_max_age}s) — skipping IMU publish.",
                    throttle_duration_sec=2.0)
                return

        imu_msg = Imu()

        imu_msg.header.frame_id = self.frame_id
        imu_msg.header.stamp = msg.system_timestamp
        
        imu_msg.orientation.x = self.q_x
        imu_msg.orientation.y= self.q_y
        imu_msg.orientation.z = self.q_z
        imu_msg.orientation.w = self.q_w

        # imu_msg.orientation.w = msg.quaternion_w
        # imu_msg.orientation.x = msg.quaternion_x
        # imu_msg.orientation.y = msg.quaternion_y
        # imu_msg.orientation.z = msg.quaternion_z

        imu_msg.linear_acceleration.x = msg.accelerometer_x
        imu_msg.linear_acceleration.y = msg.accelerometer_y
        imu_msg.linear_acceleration.z = msg.accelerometer_z

        imu_msg.angular_velocity.x = msg.gyro_x
        imu_msg.angular_velocity.y = msg.gyro_y
        imu_msg.angular_velocity.z = msg.gyro_z

        ori_cov = [0.0] * 9
        ori_var = self.orientation_noise_rad ** 2
        ori_cov[0], ori_cov[4], ori_cov[8] = ori_var, ori_var, ori_var
        imu_msg.orientation_covariance = ori_cov

         # Angular velocity covariance
        vel_cov = [0.0] * 9
        vel_var = self.angular_vel_noise ** 2
        vel_cov[0], vel_cov[4], vel_cov[8] = vel_var, vel_var, vel_var
        imu_msg.angular_velocity_covariance = vel_cov

        # Linear acceleration covariance
        acc_cov = [0.0] * 9
        acc_var = self.linear_accel_noise ** 2
        acc_cov[0], acc_cov[4], acc_cov[8] = acc_var, acc_var, acc_var
        imu_msg.linear_acceleration_covariance = acc_cov



        self.pub.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    imu_bridge = ImuBridge()
    try:
        rclpy.spin(imu_bridge)
    except KeyboardInterrupt:
        pass
    imu_bridge.destroy_node()
    rclpy.try_shutdown()

if __name__ == '__main__':
    main()