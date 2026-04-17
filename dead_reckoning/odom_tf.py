#!/usr/bin/env python3
"""
Publishes Beckholmen → map as a static TF.
Uses StaticTransformBroadcaster so the transform never expires (unlike /tf
which has a time-limited cache that breaks when sim time is paused).

XY + yaw come from /initialpose; Z comes from the latest depth_odom message.
Defaults to identity until an /initialpose is received.
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from tf2_ros import StaticTransformBroadcaster
from tf_transformations import quaternion_about_axis
class OdomTfNode(Node):

    def __init__(self):
        super().__init__('odom_tf')

        self.declare_parameter('parent_frame', 'Beckholmen')
        self.declare_parameter('child_frame',  'map')
        self.declare_parameter('depth_topic',  'depth_odom')
        self.declare_parameter('init_x', 0.0)
        self.declare_parameter('init_y', 0.0)
        self.declare_parameter('init_z', -1.5)
        self.declare_parameter('init_yaw', 0.0)

        self.parent_frame = self.get_parameter('parent_frame').get_parameter_value().string_value
        self.child_frame  = self.get_parameter('child_frame').get_parameter_value().string_value

        self.init_x = self.get_parameter('init_x').get_parameter_value().double_value
        self.init_y = self.get_parameter('init_y').get_parameter_value().double_value
        self.init_z = self.get_parameter('init_z').get_parameter_value().double_value
        self.init_yaw = self.get_parameter('init_yaw').get_parameter_value().double_value

        q = quaternion_about_axis(self.init_yaw, (0, 0, 1))
        self._br = StaticTransformBroadcaster(self)

        # Publish identity immediately so the TF tree is connected from the start
        self._publish_tf(self.init_x, self.init_y, self.init_z, q[0], q[1], q[2], q[3])

    
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self._initial_pose_cb,
            1)

        self.get_logger().info(
            f"Publishing static TF {self.parent_frame} -> {self.child_frame}.")


    def _initial_pose_cb(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._publish_tf(p.x, p.y, self.init_z, q.x, q.y, q.z, q.w)
        self.get_logger().info(
            f"Initial pose set: [{p.x:.3f}, {p.y:.3f}, {self.init_z:.3f}]")

    def _publish_tf(self, tx, ty, tz, qx, qy, qz, qw=1.0):
        t = TransformStamped()
        t.header.stamp    = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_frame
        t.child_frame_id  = self.child_frame
        t.transform.translation.x = tx
        t.transform.translation.y = ty
        t.transform.translation.z = tz
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self._br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
