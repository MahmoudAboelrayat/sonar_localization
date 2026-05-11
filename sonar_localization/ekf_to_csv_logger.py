#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import csv
import math
import os

class EkfToCsv(Node):
    def __init__(self):
        super().__init__('ekf_to_csv_logger')
        
        # Subscribe to EKF output
        self.subscription = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.callback,
            10)
            
        self.csv_file_path = 'ekf_data.csv'
        self.start_time = None
        
        # Initialize CSV file
        try:
            self.init_csv()
            self.get_logger().info(f"Logging EKF data to {os.path.abspath(self.csv_file_path)}")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize CSV: {e}")

    def init_csv(self):
        with open(self.csv_file_path, mode='w', newline='') as f:
            writer = csv.writer(f)
            # writer.writerow([
            #     'timestamp_ns', 'rel_time_sec', 
            #     'x', 'y', 'z', 
            #     'roll', 'pitch', 'yaw'
            # ])

            writer.writerow([
                'timestamp_ns', 'rel_time_sec', 
                'x', 'y', 'z', 
                'qx', 'qy', 'qz','qw'
            ])

    def quaternion_to_euler(self, q):
        # Using a more robust conversion to avoid math domain errors
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (q.w * q.y - q.z * q.x)
        # Avoid crash if sinp is slightly out of range [-1, 1] due to precision
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def callback(self, msg):
        try:
            now_ns = self.get_clock().now().nanoseconds
            if self.start_time is None:
                self.start_time = now_ns
            
            rel_time = (now_ns - self.start_time) / 1e9
            
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            z = msg.pose.pose.position.z
            
            r, p, yaw = self.quaternion_to_euler(msg.pose.pose.orientation)
            q_w = msg.pose.pose.orientation.w
            q_x = msg.pose.pose.orientation.x
            q_y =msg.pose.pose.orientation.y
            q_z = msg.pose.pose.orientation.z

            with open(self.csv_file_path, mode='a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([now_ns, rel_time, x, y, z, q_x, q_y, q_z,q_w])
                
        except Exception as e:
            self.get_logger().error(f"Callback crashed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = EkfToCsv()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()