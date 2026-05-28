#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from interfaces.msg import IMU, AHRS
from sensor_msgs.msg import Imu
import numpy as np

class AHRSBridge(Node):
    def __init__(self):
        super().__init__("ahrs_bridge")
        self.get_logger().info("AHRS bridge started")

        self.declare_parameter('frame_id', 'dvl_link')
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.declare_parameter('ahrs_topic', '/nucleus_node/ahrs_packets')
        self.ahrs_topic = self.get_parameter('ahrs_topic').get_parameter_value().string_value
        self.declare_parameter('output_topic', '/dvl/ahrs_imu')
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value


        self.ahrs_Sub = self.create_subscription(AHRS, self.ahrs_topic, self.ahrs_callback, 10)

        self.pub = self.create_publisher(Imu, self.output_topic, 10)
        self.linear_accel_noise = 9.4e-8  # m/s^2
        self.angular_vel_noise = 5.7e-8   # rad/s
        self.orientation_noise_deg = 5e-8 # Degrees of jitter
        self.orientation_noise_rad = np.radians(self.orientation_noise_deg)



        self.q_w = 1.0
        self.q_x = 0.0
        self.q_y = 0.0
        self.q_z = 0.0


    def ahrs_callback(self, msg):
        # if not msg.is_valid:
        #     self.get_logger().warn("Invalid AHRS packet, skipping.")
        #     return


        imu_msg = Imu()

        imu_msg.header.frame_id = self.frame_id
        imu_msg.header.stamp = msg.system_timestamp
        
        imu_msg.orientation.x = msg.quaternion_x
        imu_msg.orientation.y= msg.quaternion_y
        imu_msg.orientation.z = msg.quaternion_z
        imu_msg.orientation.w = msg.quaternion_w

        imu_msg.linear_acceleration.x = 0.0
        imu_msg.linear_acceleration.y = 0.0
        imu_msg.linear_acceleration.z = 0.0

        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0    
        imu_msg.angular_velocity.z = 0.0

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
    ahrs_bridge = AHRSBridge()
    try:
        rclpy.spin(ahrs_bridge)
    except KeyboardInterrupt:
        pass
    ahrs_bridge.destroy_node()
    rclpy.try_shutdown()

if __name__ == '__main__':
    main()