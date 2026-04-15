#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Point
import csv
import time

class CsvOdomPublisher(Node):
    def __init__(self):
        super().__init__('csv_odom_publisher')
        self.publisher_ = self.create_publisher(Odometry, 'odom', 10)
        
        # Path to your CSV file
        self.csv_file_path = '/home/mahmoud/thesis/scripts/build/refined_trajectory.csv'
        self.publish_from_csv()

    def publish_from_csv(self):
        try:
            with open(self.csv_file_path, mode='r') as f:
                reader = csv.DictReader(f)
                self.get_logger().info(f"Starting playback from {self.csv_file_path}")

                for row in reader:
                    if not rclpy.ok():
                        break

                    msg = Odometry()
                    
                    # 1. Header & Timestamp
                    timestamp = float(row['timestamp'])
                    msg.header.stamp.sec = int(timestamp*1e-9)
                    msg.header.stamp.nanosec = int((timestamp*1e-9 - int(timestamp)*1e-9) * 1e9)
                    msg.header.frame_id = 'Beckholmen'
                    msg.child_frame_id = 'saabmarine/base_link'

                    msg.pose.pose.position = Point(
                        x=float(row['tx']), 
                        y=float(row['ty']), 
                        z=float(row['tz'])
                    )

                    msg.pose.pose.orientation = Quaternion(
                        x=float(row['qx']), 
                        y=float(row['qy']), 
                        z=float(row['qz']), 
                        w=float(row['qw'])
                    )

                    self.publisher_.publish(msg)
                    
                    # Log progress occasionally
                    # self.get_logger().info(f"Published odom for time: {timestamp}")

                    # Simulate real-time (adjust to your LiDAR frequency, e.g., 10Hz = 0.1)
                    time.sleep(0.1) 

        except FileNotFoundError:
            self.get_logger().error(f"CSV file not found at {self.csv_file_path}")
        except Exception as e:
            self.get_logger().error(f"Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CsvOdomPublisher()
    # No need for rclpy.spin() if the loop is in __init__, 
    # but normally you'd use a timer for the loop.
    rclpy.shutdown()

if __name__ == '__main__':
    main()