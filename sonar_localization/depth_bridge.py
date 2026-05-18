#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped

class DepthToOdom(Node):
    def __init__(self):
        super().__init__("depth_to_odom")
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.declare_parameter("global_pos_topic", "global_position/global")
        self.declare_parameter("rel_alt_topic", "/sam_auv_v1/smarc/dep")
        self.declare_parameter("output_topic", "depth_odom")
        self.declare_parameter("relative_depth", True)
        self.declare_parameter("ned", False)

        global_pos_topic = self.get_parameter("global_pos_topic").value
        rel_alt_topic = self.get_parameter("rel_alt_topic").value
        output_topic = self.get_parameter("output_topic").value
        self.relative_depth = self.get_parameter("relative_depth").get_parameter_value().bool_value
        self.ned = self.get_parameter("ned").get_parameter_value().bool_value
        
        self.declare_parameter("sim", True)
        self.sim = self.get_parameter("sim").get_parameter_value().bool_value

        self.declare_parameter("frame_id", "base_link")
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value

        self.declare_parameter("depth_cov", 1e-4)
        self.depth_cov = self.get_parameter("depth_cov").get_parameter_value().double_value

        self.stamp = None
        self.depth_offset = None

        self.pos_sub = self.create_subscription(
            NavSatFix,
            global_pos_topic,
            self.global_pos_callback,
            qos_profile=qos_profile
        )
        self.sub = self.create_subscription(
            Float64,
            rel_alt_topic,
            self.rel_alt_callback,
            qos_profile=qos_profile
        )

        self.pub = self.create_publisher(PoseWithCovarianceStamped, output_topic, 10)

        self.get_logger().info(
            f"Depth-to-Odometry converter started.\n"
            f"Listening to: {rel_alt_topic}\n"
            f"Publishing:   {output_topic}"
        )
    def global_pos_callback(self, msg: NavSatFix):
        self.stamp = msg.header.stamp

    def rel_alt_callback(self, msg: Float64):
        if self.ned:
            rel_alt = -msg.data  # Convert NED to ENU
        else:
            rel_alt = msg.data  # in meters

        if self.depth_offset is None and self.relative_depth:
            self.depth_offset = rel_alt
            self.get_logger().info(f"Depth offset set to {self.depth_offset:.3f} m")
        elif not self.relative_depth:
            self.depth_offset = 0.0

        depth = rel_alt - self.depth_offset

        odom = PoseWithCovarianceStamped()
        odom.header.stamp = self.stamp if self.stamp else self.get_clock().now().to_msg()
        # odom.header.frame_id = "Beckholmen"
        odom.header.frame_id = self.frame_id

        odom.pose.pose.position.x = 0.0
        odom.pose.pose.position.y = 0.0
        odom.pose.pose.position.z = -depth

        odom.pose.covariance = [
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, self.depth_cov, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        ]

        self.pub.publish(odom)


def main():
    rclpy.init()
    node = DepthToOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()