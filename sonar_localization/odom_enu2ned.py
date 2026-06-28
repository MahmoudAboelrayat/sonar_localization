#!/usr/bin/env python3
"""
odom_enu2ned.py

Relay node: converts an Odometry message from ENU to NED frame.

ENU → NED:
  X_NED =  Y_ENU  (North)
  Y_NED =  X_ENU  (East)
  Z_NED = -Z_ENU  (Down)

Rotation quaternion:  q = [w=0, x=√2/2, y=√2/2, z=0]
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import Odometry

_R = np.array([
    [1.0,  0.0,  0.0],
    [0.0,  -1.0,  0.0],
    [0.0,  0.0, -1.0],
], dtype=np.float64)

_s = math.sqrt(2.0) / 2.0
_Q = np.array([0.0, _s, _s, 0.0])  # [w, x, y, z]

_R6 = np.zeros((6, 6))
_R6[0:3, 0:3] = _R
_R6[3:6, 3:6] = _R


def _qmul(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ])


class OdomENU2NED(Node):
    def __init__(self):
        super().__init__('odom_enu2ned')

        self.declare_parameter('input_topic',  '/odometry/gps')
        self.declare_parameter('output_topic', '/odometry/gps_ned')
        self.declare_parameter('output_frame', 'odom_ned')

        in_topic         = self.get_parameter('input_topic').value
        out_topic        = self.get_parameter('output_topic').value
        self.out_frame   = self.get_parameter('output_frame').value

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.sub = self.create_subscription(Odometry, in_topic,  self.callback, qos)
        self.pub = self.create_publisher(Odometry,    out_topic, 10)

        self.get_logger().info(f'odom_enu2ned: {in_topic} → {out_topic}')

    def callback(self, msg: Odometry):
        out = Odometry()
        out.header             = msg.header
        out.header.frame_id    = self.out_frame
        out.child_frame_id     = msg.child_frame_id

        # Position
        p = np.array([msg.pose.pose.position.x,
                      msg.pose.pose.position.y,
                      msg.pose.pose.position.z])
        pn = _R @ p
        out.pose.pose.position.x = pn[0]
        out.pose.pose.position.y = pn[1]
        out.pose.pose.position.z = pn[2]

        # Orientation
        # q = np.array([msg.pose.pose.orientation.w,
        #               msg.pose.pose.orientation.x,
        #               msg.pose.pose.orientation.y,
        #               msg.pose.pose.orientation.z])
        # qn = _qmul(_Q, q)
        out.pose.pose.orientation.w = msg.pose.pose.orientation.w
        out.pose.pose.orientation.x = msg.pose.pose.orientation.x
        out.pose.pose.orientation.y = msg.pose.pose.orientation.y
        out.pose.pose.orientation.z = msg.pose.pose.orientation.z

        # Pose covariance  (R6 @ C @ R6ᵀ)
        C = np.array(msg.pose.covariance).reshape(6, 6)
        out.pose.covariance = (_R6 @ C @ _R6.T).flatten().tolist()

        # Linear velocity
        v = np.array([msg.twist.twist.linear.x,
                      msg.twist.twist.linear.y,
                      msg.twist.twist.linear.z])
        vn = _R @ v
        out.twist.twist.linear.x = vn[0]
        out.twist.twist.linear.y = vn[1]
        out.twist.twist.linear.z = vn[2]

        # Angular velocity
        w = np.array([msg.twist.twist.angular.x,
                      msg.twist.twist.angular.y,
                      msg.twist.twist.angular.z])
        wn = _R @ w
        out.twist.twist.angular.x = wn[0]
        out.twist.twist.angular.y = wn[1]
        out.twist.twist.angular.z = wn[2]

        # Twist covariance
        T = np.array(msg.twist.covariance).reshape(6, 6)
        out.twist.covariance = (_R6 @ T @ _R6.T).flatten().tolist()

        self.pub.publish(out)


def main():
    rclpy.init()
    node = OdomENU2NED()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
