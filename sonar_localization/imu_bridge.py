#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
import random
from scipy.spatial.transform import Rotation as R

class ImuNoiseBridge(Node):

    def __init__(self):
        super().__init__("imu_noise_bridge")
        self.get_logger().info("IMU Noise Bridge Started")
        # self.declare_parameter('imu_raw_topic', '/BlueROV2_Heavy/mavros/imu/data_raw')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('output_topic','/imu/data_cov')
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value

        # Subscribing to the raw/clean IMU from simulator
        self.sub = self.create_subscription(Imu, self.imu_topic, self.callback, 10)
        # Publishing noisy IMU for the EKF
        self.pub = self.create_publisher(Imu, self.output_topic, 10)

        self.declare_parameter('frame_id', 'imu_link_ned')
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        
        self.declare_parameter('add_noise', False)
        self.add_noise = self.get_parameter('add_noise').get_parameter_value().bool_value

        # CONFIGURATION: Set your noise levels (Standard Deviations)
        self.linear_accel_noise = 9.4e-8  # m/s^2
        self.angular_vel_noise = 5.7e-8   # rad/s
        self.orientation_noise_deg = 5e-8 # Degrees of jitter

        # self.angular_vel_noise = 1e-1   # rad/s
        # self.orientation_noise_deg = 1e-1 # Degrees of jitter
        
        self.orientation_noise_rad = np.radians(self.orientation_noise_deg)

    def callback(self, msg):
        noisy_msg = Imu()
        noisy_msg.header = msg.header
        # Ensure the frame matches your robot's URDF/TF tree
        noisy_msg.header.frame_id = self.frame_id

        # 1. ADD NOISE TO LINEAR ACCELERATION (x, y, z)
        if (self.add_noise):
            noisy_msg.linear_acceleration.x = msg.linear_acceleration.x + random.gauss(0, self.linear_accel_noise)
            noisy_msg.linear_acceleration.y = msg.linear_acceleration.y + random.gauss(0, self.linear_accel_noise)
            noisy_msg.linear_acceleration.z = msg.linear_acceleration.z + random.gauss(0, self.linear_accel_noise)

            # 2. ADD NOISE TO ANGULAR VELOCITY (Roll, Pitch, Yaw rates)
            noisy_msg.angular_velocity.x = msg.angular_velocity.x + random.gauss(0, self.angular_vel_noise)
            noisy_msg.angular_velocity.y = msg.angular_velocity.y + random.gauss(0, self.angular_vel_noise)
            noisy_msg.angular_velocity.z = msg.angular_velocity.z + random.gauss(0, self.angular_vel_noise)

            # 3. ADD NOISE TO ORIENTATION (Quaternion math)
            # Create a small random rotation (the "error")
            roll_err = random.gauss(0, self.orientation_noise_rad)
            pitch_err = random.gauss(0, self.orientation_noise_rad)
            yaw_err = random.gauss(0, self.orientation_noise_rad)
            error_rot = R.from_euler('xyz', [roll_err, pitch_err, yaw_err])

            # Get original orientation
            original_rot = R.from_quat([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])

            # Apply error: New = Original * Error
            combined_rot = original_rot * error_rot
            q = combined_rot.as_quat() # returns [x, y, z, w]

            noisy_msg.orientation.x = q[0]
            noisy_msg.orientation.y = q[1]
            noisy_msg.orientation.z = q[2]
            noisy_msg.orientation.w = q[3]
        else:
            noisy_msg.linear_acceleration =msg.linear_acceleration
            noisy_msg.orientation = msg.orientation
            noisy_msg.angular_velocity = msg.angular_velocity

        # 4. UPDATE COVARIANCES (Variance = sigma^2)
        # Orientation covariance [roll, pitch, yaw]
        ori_cov = [0.0] * 9
        ori_var = self.orientation_noise_rad ** 2
        ori_cov[0], ori_cov[4], ori_cov[8] = ori_var, ori_var, ori_var
        noisy_msg.orientation_covariance = ori_cov

        # Angular velocity covariance
        vel_cov = [0.0] * 9
        vel_var = self.angular_vel_noise ** 2
        vel_cov[0], vel_cov[4], vel_cov[8] = vel_var, vel_var, vel_var
        noisy_msg.angular_velocity_covariance = vel_cov

        # Linear acceleration covariance
        acc_cov = [0.0] * 9
        acc_var = self.linear_accel_noise ** 2
        acc_cov[0], acc_cov[4], acc_cov[8] = acc_var, acc_var, acc_var
        noisy_msg.linear_acceleration_covariance = acc_cov

        self.pub.publish(noisy_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuNoiseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()

if __name__ == '__main__':
    main()