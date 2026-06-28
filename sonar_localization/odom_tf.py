#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from tf2_ros import StaticTransformBroadcaster
# from tf_transformations import quaternion_from_euler, euler_from_quaternion
import numpy as np
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
        self.declare_parameter('init_roll', 0.0)
        self.declare_parameter('init_pitch', 0.0)
        self.declare_parameter('lock_roll_pitch', True)
        self.declare_parameter('lock_z', True)
        self.parent_frame = self.get_parameter('parent_frame').get_parameter_value().string_value
        self.child_frame  = self.get_parameter('child_frame').get_parameter_value().string_value

        self.init_x = self.get_parameter('init_x').get_parameter_value().double_value
        self.init_y = self.get_parameter('init_y').get_parameter_value().double_value
        self.init_z = self.get_parameter('init_z').get_parameter_value().double_value
        self.init_yaw = self.get_parameter('init_yaw').get_parameter_value().double_value
        self.init_roll       = self.get_parameter('init_roll').get_parameter_value().double_value
        self.init_pitch      = self.get_parameter('init_pitch').get_parameter_value().double_value
        self.lock_roll_pitch = self.get_parameter('lock_roll_pitch').get_parameter_value().bool_value
        self.lock_z          = self.get_parameter('lock_z').get_parameter_value().bool_value

        q = self.quaternion_from_euler(self.init_roll, self.init_pitch, self.init_yaw)
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

    def euler_from_quaternion(self, q):
        # Using a more robust conversion to avoid math domain errors
        qx = q[0]
        qy = q[1]
        qz = q[2]
        qw = q[3]
        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (qw * qy - qz * qx)
        # Avoid crash if sinp is slightly out of range [-1, 1] due to precision
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def quaternion_from_euler(self,roll, pitch, yaw):
        """
        Convert Euler angles (roll, pitch, yaw) to quaternion (x, y, z, w)
        """
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return np.array([qx, qy, qz, qw])

    def _initial_pose_cb(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        roll, pitch, yaw = self.euler_from_quaternion([q.x, q.y, q.z, q.w])
        r_out = self.init_roll  if self.lock_roll_pitch else roll
        p_out = self.init_pitch if self.lock_roll_pitch else pitch
        z_out = self.init_z     if self.lock_z          else p.z
        q_fixed = self.quaternion_from_euler(r_out, p_out, yaw)
        self._publish_tf(p.x, p.y, z_out, q_fixed[0], q_fixed[1], q_fixed[2], q_fixed[3])
        self.get_logger().info(
            f"Initial pose set: [{p.x:.3f}, {p.y:.3f}, {z_out:.3f}], [{roll:.3f}, {pitch:.3f}, {yaw:.3f}]")

    def _publish_tf(self, tx, ty, tz, qx, qy, qz, qw=1.0):
        t = TransformStamped()
        t.header.stamp    = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_frame
        t.child_frame_id  = self.child_frame
        t.transform.translation.x = tx
        t.transform.translation.y = ty
        t.transform.translation.z = -tz
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
