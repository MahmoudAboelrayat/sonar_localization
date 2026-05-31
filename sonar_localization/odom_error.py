#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import message_filters
import numpy as np
import math


def quat_to_rot(q):
    """Quaternion (x,y,z,w) → 3×3 rotation matrix."""
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1-2*(y*y+z*z),   2*(x*y-z*w),   2*(x*z+y*w)],
        [  2*(x*y+z*w), 1-2*(x*x+z*z),   2*(y*z-x*w)],
        [  2*(x*z-y*w),   2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def pose_to_mat(msg_pose):
    """nav_msgs Pose → 4×4 SE(3) matrix."""
    p = msg_pose.position
    R = quat_to_rot(msg_pose.orientation)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3,  3] = [p.x, p.y, p.z]
    return T


def rotation_error_deg(R1, R2):
    """Geodesic angle [deg] between two rotation matrices."""
    R_rel = R1.T @ R2
    cos_angle = (np.trace(R_rel) - 1.0) / 2.0
    cos_angle = max(-1.0, min(1.0, cos_angle))
    return math.degrees(math.acos(cos_angle))


def print_stats(label, trans_errors, rot_errors, n):
    avg_t  = np.mean(trans_errors)
    avg_r  = np.mean(rot_errors)
    rmse_t = math.sqrt(np.mean(np.square(trans_errors)))
    rmse_r = math.sqrt(np.mean(np.square(rot_errors)))
    max_t  = np.max(trans_errors)
    max_r  = np.max(rot_errors)
    return (
        f'\n{label} ({n} samples)\n'
        f'  Translation  avg: {avg_t:.4f} m   RMSE: {rmse_t:.4f} m   max: {max_t:.4f} m\n'
        f'  Rotation     avg: {avg_r:.4f}°   RMSE: {rmse_r:.4f}°   max: {max_r:.4f}°'
    )


class OdomError(Node):
    def __init__(self):
        super().__init__('odom_error')

        topic_a            = self.declare_parameter('topic_a',         '/vgicp_odom').value
        topic_b            = self.declare_parameter('topic_b',         '/odometry/filtered').value
        self.print_interval = self.declare_parameter('print_interval', 10).value
        self.max_time_diff  = self.declare_parameter('max_time_diff',  0.3).value

        self.get_logger().info(f'Comparing:\n  A: {topic_a}\n  B: {topic_b}')

        sub_a = message_filters.Subscriber(self, Odometry, topic_a)
        sub_b = message_filters.Subscriber(self, Odometry, topic_b)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [sub_a, sub_b], queue_size=50, slop=self.max_time_diff)
        self.sync.registerCallback(self.callback)

        # absolute error accumulators
        self.abs_trans = []
        self.abs_rot   = []

        # relative-to-first error accumulators
        self.rel_trans = []
        self.rel_rot   = []

        # anchors (set on first message pair)
        self.T_a0 = None
        self.T_b0 = None

        self.n = 0

    def callback(self, msg_a: Odometry, msg_b: Odometry):
        Ta = pose_to_mat(msg_a.pose.pose)
        Tb = pose_to_mat(msg_b.pose.pose)

        # ── Anchor to first message ───────────────────────────────────────────
        if self.T_a0 is None:
            self.T_a0 = Ta.copy()
            self.T_b0 = Tb.copy()
            self.get_logger().info('Anchors set from first synchronized pair.')
            return

        # ── Absolute error (raw poses) ────────────────────────────────────────
        abs_t_err = np.linalg.norm(Ta[:3, 3] - Tb[:3, 3])
        abs_r_err = rotation_error_deg(Ta[:3, :3], Tb[:3, :3])
        self.abs_trans.append(abs_t_err)
        self.abs_rot.append(abs_r_err)

        # ── Relative error (motion since first pose) ──────────────────────────
        # Express each trajectory as displacement from its own origin,
        # then compare those displacements — frame-independent.
        Ta_rel = np.linalg.inv(self.T_a0) @ Ta
        Tb_rel = np.linalg.inv(self.T_b0) @ Tb
        rel_t_err = np.linalg.norm(Ta_rel[:3, 3] - Tb_rel[:3, 3])
        rel_r_err = rotation_error_deg(Ta_rel[:3, :3], Tb_rel[:3, :3])
        self.rel_trans.append(rel_t_err)
        self.rel_rot.append(rel_r_err)

        self.n += 1

        if self.n % self.print_interval == 0:
            self.get_logger().info(
                print_stats('--- Absolute error', self.abs_trans, self.abs_rot, self.n) +
                print_stats('\n--- Relative error (frame-independent)', self.rel_trans, self.rel_rot, self.n)
            )


def main(args=None):
    rclpy.init(args=args)
    node = OdomError()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.n > 0:
            print(
                print_stats('=== Final absolute error', node.abs_trans, node.abs_rot, node.n) +
                print_stats('\n=== Final relative error (frame-independent)', node.rel_trans, node.rel_rot, node.n)
            )
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
