#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from interfaces.msg import BottomTrack
from geometry_msgs.msg import TwistWithCovarianceStamped
from smarc_msgs.msg import DVL
import random

class DvlBridge(Node):

    def __init__(self):
        super().__init__("dvl_bridge")
        self.get_logger().info("DVL bridge")

        self.declare_parameter('frame_id', 'dvl_link')
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        self.declare_parameter('input_topic', '/nucleus_node/bottom_track_packets')
        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value

        self.sub = self.create_subscription(BottomTrack, self.input_topic, self.callback, 10)
        self.pub = self.create_publisher(TwistWithCovarianceStamped, '/dvl/velocity', 10)

        self.cov = [0.0] * 36

    def callback(self, msg):
        publish_msg = TwistWithCovarianceStamped()
        publish_msg.header.frame_id = self.frame_id
        publish_msg.header.stamp = msg.system_timestamp

        publish_msg.twist.twist.linear.x = msg.velocity_x
        publish_msg.twist.twist.linear.y = msg.velocity_y
        publish_msg.twist.twist.linear.z = msg.velocity_z

        self.cov[0] = msg.fom_x
        self.cov[7] = msg.fom_y
        self.cov[14] = msg.fom_z

        publish_msg.twist.covariance = self.cov
        self.pub.publish(publish_msg)
    

def main(args=None):
    rclpy.init(args=args)

    dvl_bridge = DvlBridge()

    try:
        rclpy.spin(dvl_bridge)
    except KeyboardInterrupt:
        pass

    dvl_bridge.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
