#!/usr/bin/env python3
"""
Ground Truth Map Builder
========================
Subscribes : /sonar/point_cloud     (sensor_msgs/PointCloud2)
             /odometry/ground_truth  (nav_msgs/Odometry)  ← sim ground truth
Publishes  : /gt_map                (sensor_msgs/PointCloud2)
Saves      : gt_map.pcd             when node shuts down

How it works:
  For each incoming sonar scan:
    1. Get the latest ground truth pose (position + orientation)
    2. Transform the scan from sensor frame → world frame using that pose
    3. Accumulate into a global map
    4. Downsample periodically to keep memory manageable
"""

import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
import threading

try:
    import open3d as o3d
    HAS_OPEN3D = True
except ImportError:
    HAS_OPEN3D = False
    print("open3d not found — map will not be saved as .pcd on shutdown")


class GroundTruthMapBuilder(Node):

    def __init__(self):
        super().__init__('gt_map_builder')

        # ── Parameters ────────────────────────────────────────
        self.declare_parameter('point_cloud_topic',  '/sonar/point_cloud_local')
        self.declare_parameter('odom_topic',         '/BlueROV2_Heavy/odom_gt')
        self.declare_parameter('map_topic',          '/gt_map')
        self.declare_parameter('world_frame',        'unity_origin')
        self.declare_parameter('voxel_size',         0.05)   # metres — map resolution
        self.declare_parameter('publish_every_n',    5)      # publish map every N scans
        self.declare_parameter('downsample_every_n', 20)     # voxel downsample every N scans
        self.declare_parameter('save_path',          'gt_map.pcd')

        # Sonar extrinsics (sensor → base_link)
        # Change these to match your robot's mounting
        self.declare_parameter('sonar2base_x',     0.007)
        self.declare_parameter('sonar2base_y',      0.000)
        self.declare_parameter('sonar2base_z',     -0.091)
        self.declare_parameter('sonar2base_roll',   0.0)
        self.declare_parameter('sonar2base_pitch', -30.0)   # degrees
        self.declare_parameter('sonar2base_yaw',    0.0)

        self.voxel_size_      = self.get_parameter('voxel_size').value
        self.publish_every_n_ = self.get_parameter('publish_every_n').value
        self.downsample_n_    = self.get_parameter('downsample_every_n').value
        self.save_path_       = self.get_parameter('save_path').value
        self.world_frame_     = self.get_parameter('world_frame').value

        # ── Build sonar→base_link transform matrix ────────────
        s2b_x   = self.get_parameter('sonar2base_x').value
        s2b_y   = self.get_parameter('sonar2base_y').value
        s2b_z   = self.get_parameter('sonar2base_z').value
        s2b_r   = np.deg2rad(self.get_parameter('sonar2base_roll').value)
        s2b_p   = np.deg2rad(self.get_parameter('sonar2base_pitch').value)
        s2b_yaw = np.deg2rad(self.get_parameter('sonar2base_yaw').value)

        R_s2b = Rotation.from_euler('zyx', [s2b_yaw, s2b_p, s2b_r]).as_matrix()
        self.T_sonar2base_ = np.eye(4)
        self.T_sonar2base_[:3, :3] = R_s2b
        self.T_sonar2base_[:3,  3] = [s2b_x, s2b_y, s2b_z]
        self.T_base2sonar_ = np.linalg.inv(self.T_sonar2base_)

        self.get_logger().info(
            f'Sonar→base transform:\n{self.T_sonar2base_}')

        # ── State ─────────────────────────────────────────────
        self.latest_odom_   = None   # latest ground truth pose as 4x4 matrix
        self.odom_lock_     = threading.Lock()
        self.global_map_    = np.empty((0, 3), dtype=np.float32)
        self.scan_count_    = 0

        # ── Subscribers ───────────────────────────────────────
        self.odom_sub_ = self.create_subscription(
            Odometry,
            self.get_parameter('odom_topic').value,
            self.odom_callback,
            10)

        self.pc_sub_ = self.create_subscription(
            PointCloud2,
            self.get_parameter('point_cloud_topic').value,
            self.cloud_callback,
            10)

        # ── Publisher ─────────────────────────────────────────
        self.map_pub_ = self.create_publisher(
            PointCloud2,
            self.get_parameter('map_topic').value,
            1)

        self.get_logger().info('Ground truth map builder started.')
        self.get_logger().info(
            f'Subscribing to: {self.get_parameter("point_cloud_topic").value}')
        self.get_logger().info(
            f'Subscribing to: {self.get_parameter("odom_topic").value}')

    # ──────────────────────────────────────────────────────────
    #  Odometry callback — just store the latest pose
    # ──────────────────────────────────────────────────────────
    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        # Quaternion → rotation matrix
        rot = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()

        T = np.eye(4)
        T[:3, :3] = rot
        T[:3,  3] = [p.x, p.y, p.z]

        with self.odom_lock_:
            self.latest_odom_ = T

    # ──────────────────────────────────────────────────────────
    #  Point cloud callback — transform and accumulate
    # ──────────────────────────────────────────────────────────
    def cloud_callback(self, msg: PointCloud2):
        # ── Get latest ground truth pose ──────────────────────
        with self.odom_lock_:
            if self.latest_odom_ is None:
                self.get_logger().warn_once(
                    'Waiting for first odometry message...')
                return
            T_base2world = self.latest_odom_.copy()

        # ── Convert PointCloud2 → numpy (N,3) ─────────────────
        raw = list(pc2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True))
        if not raw:
            return
        points_sensor = np.array(
            [[x, y, z] for x, y, z in raw], dtype=np.float32)   # (N,3)

        # ── Transform: sensor → base_link → world ─────────────
        #
        # T_sonar2world = T_base2world @ T_sonar2base
        #
        T_sonar2world = T_base2world @ self.T_sonar2base_

        # Apply transform: p_world = R @ p_sensor + t
        R = T_sonar2world[:3, :3].astype(np.float32)
        t = T_sonar2world[:3,  3].astype(np.float32)
        points_world = (R @ points_sensor.T).T + t   # (N,3)

        # ── Accumulate into global map ─────────────────────────
        self.global_map_ = np.vstack([self.global_map_, points_world])
        self.scan_count_ += 1

        self.get_logger().debug(
            f'Scan {self.scan_count_}: added {len(points_world)} pts, '
            f'map total: {len(self.global_map_):,}')

        # ── Periodic voxel downsample ──────────────────────────
        if self.scan_count_ % self.downsample_n_ == 0:
            self.global_map_ = self.voxel_downsample(
                self.global_map_, self.voxel_size_)
            self.get_logger().info(
                f'Downsampled map: {len(self.global_map_):,} pts')

        # ── Periodic publish ───────────────────────────────────
        if self.scan_count_ % self.publish_every_n_ == 0:
            self.publish_map(msg.header.stamp)

    # ──────────────────────────────────────────────────────────
    #  Voxel downsampling (pure numpy — no PCL needed)
    # ──────────────────────────────────────────────────────────
    def voxel_downsample(self, points: np.ndarray,
                          voxel_size: float) -> np.ndarray:
        if len(points) == 0:
            return points

        # Assign each point to a voxel index
        voxel_idx = np.floor(points / voxel_size).astype(np.int32)

        # Use a dict to keep one point per voxel (the first one seen)
        voxel_dict = {}
        for i, idx in enumerate(map(tuple, voxel_idx)):
            if idx not in voxel_dict:
                voxel_dict[idx] = i

        kept = np.array(list(voxel_dict.values()))
        return points[kept]

    # ──────────────────────────────────────────────────────────
    #  Publish the accumulated map
    # ──────────────────────────────────────────────────────────
    def publish_map(self, stamp):
        if len(self.global_map_) == 0:
            return

        header = Header()
        header.stamp    = stamp
        header.frame_id = self.world_frame_

        fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]

        msg = pc2.create_cloud(header, fields, self.global_map_.tolist())
        self.map_pub_.publish(msg)

    # ──────────────────────────────────────────────────────────
    #  Save map on shutdown
    # ──────────────────────────────────────────────────────────
    def save_map(self):
        if len(self.global_map_) == 0:
            self.get_logger().warn('Map is empty — nothing to save.')
            return

        # Final downsample before saving
        final_map = self.voxel_downsample(self.global_map_, self.voxel_size_)
        self.get_logger().info(
            f'Saving map: {len(final_map):,} points → {self.save_path_}')

        if HAS_OPEN3D:
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(final_map.astype(np.float64))
            o3d.io.write_point_cloud(self.save_path_, pcd)
            self.get_logger().info(f'Map saved to {self.save_path_}')
        else:
            # Fallback: save as plain text CSV
            csv_path = self.save_path_.replace('.pcd', '.csv')
            np.savetxt(csv_path, final_map, delimiter=',', header='x,y,z')
            self.get_logger().info(f'Map saved as CSV to {csv_path}')


# ──────────────────────────────────────────────────────────────
#  Entry point
# ──────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthMapBuilder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_map()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()