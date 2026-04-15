#!/usr/bin/env python3
"""
Sonar Surface Feature Extraction – ROS2 Python Node
=====================================================
Subscribes : /sonar/point_cloud  (sensor_msgs/PointCloud2)
Publishes  : /sonar/surface_features  (sensor_msgs/PointCloud2)  – mean points Pm
             /sonar/surface_normals   (visualization_msgs/MarkerArray) – normals um
"""

import numpy as np
from collections import defaultdict

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import Header, ColorRGBA

import sensor_msgs_py.point_cloud2 as pc2


# ──────────────────────────────────────────────────────────────
#  Helpers
# ──────────────────────────────────────────────────────────────

def xyz_to_voxel_key(point: np.ndarray, voxel_size: float):
    """Map a 3-D point to an integer voxel key (ix, iy, iz)."""
    return tuple(np.floor(point / voxel_size).astype(int))


def compute_surface_features(points: np.ndarray,
                              voxel_size: float,
                              gamma: int):
    """
    Parameters
    ----------
    points     : (N, 3) float32 array  – raw XYZ point cloud
    voxel_size : float                 – side length of each voxel [m]
    gamma      : int                   – minimum neighbour count threshold

    Returns
    -------
    means   : (M, 3) – representative point  Pm  per feature voxel
    normals : (M, 3) – surface normal         um  per feature voxel
    """

    # ── STEP 1: Partition into voxels ────────────────────────
    voxel_map = defaultdict(list)   # key → list of point indices
    for i, pt in enumerate(points):
        key = xyz_to_voxel_key(pt, voxel_size)
        voxel_map[key].append(i)

    means   = []
    normals = []

    offsets = [
        (dx, dy, dz)
        for dx in (-1, 0, 1)
        for dy in (-1, 0, 1)
        for dz in (-1, 0, 1)
    ]  # 27 entries including (0,0,0) — the voxel itself

    for key in voxel_map:

        # ── STEP 2: Gather neighbours from 26 adjacent voxels ──
        neighbour_pts = []
        kx, ky, kz = key
        for dx, dy, dz in offsets:
            nb = (kx + dx, ky + dy, kz + dz)
            if nb in voxel_map:
                for idx in voxel_map[nb]:
                    neighbour_pts.append(points[idx])

        # ── STEP 3: Threshold filter ──────────────────────────
        if len(neighbour_pts) < gamma:
            continue
        nb_array =np.array([[x, y, z] for x, y, z in neighbour_pts], dtype=np.float32)

        # ── STEP 4: Mean point  Pm ────────────────────────────
        pm = nb_array.mean(axis=0)

        # ── STEP 5: PCA → normal  um ──────────────────────────
        #   Covariance matrix of centred points
        centred = nb_array - pm                       # (K, 3)
        cov     = (centred.T @ centred) / len(nb_array)   # (3, 3)

        #   Eigendecomposition – eigenvalues sorted ascending
        eigenvalues, eigenvectors = np.linalg.eigh(cov)
        # eigenvectors[:,i] corresponds to eigenvalues[i]
        # smallest eigenvalue → eigenvectors[:,0] = surface normal
        um = eigenvectors[:, 0]

        means.append(pm)
        normals.append(um)

    if not means:
        return np.empty((0, 3)), np.empty((0, 3))

    return np.array(means, dtype=np.float32), np.array(normals, dtype=np.float32)


# ──────────────────────────────────────────────────────────────
#  ROS2 Node
# ──────────────────────────────────────────────────────────────

class SonarSurfaceFeatures(Node):

    def __init__(self):
        super().__init__('sonar_surface_features')

        # ── parameters ────────────────────────────────────────
        self.declare_parameter('voxel_size',       1.0)
        self.declare_parameter('gamma',            10)
        self.declare_parameter('publish_markers',  True)

        self.voxel_size      = self.get_parameter('voxel_size').value
        self.gamma           = self.get_parameter('gamma').value
        self.publish_markers = self.get_parameter('publish_markers').value

        self.get_logger().info(
            f'voxel_size={self.voxel_size:.3f}  gamma={self.gamma}')

        # ── subscriber ────────────────────────────────────────
        self.sub = self.create_subscription(
            PointCloud2,
            '/sonar/point_cloud',
            self.cloud_callback,
            10)

        # ── publishers ────────────────────────────────────────
        self.feat_pub = self.create_publisher(
            PointCloud2, '/sonar/surface_features', 10)

        self.marker_pub = None
        if self.publish_markers:
            self.marker_pub = self.create_publisher(
                MarkerArray, '/sonar/surface_normals', 10)

    # ──────────────────────────────────────────────────────────
    #  Callback
    # ──────────────────────────────────────────────────────────

    def cloud_callback(self, msg: PointCloud2):
        raw = list(pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True))

        if not raw:
            self.get_logger().warn('Empty point cloud – skipping.')
            return

        points = np.array([[x, y, z] for x, y, z in raw], dtype=np.float32)  # (N,3)

        # ── Extract surface features ───────────────────────────
        means, normals = compute_surface_features(
            points, self.voxel_size, self.gamma)

        self.get_logger().info(
            f'Extracted {len(means)} surface features from {len(points)} points')

        # ── Publish ────────────────────────────────────────────
        self._publish_feature_cloud(means, msg.header)

        if self.publish_markers and self.marker_pub is not None:
            self._publish_normal_markers(means, normals, msg.header)

    # ──────────────────────────────────────────────────────────
    #  Publish mean points as PointCloud2
    # ──────────────────────────────────────────────────────────

    def _publish_feature_cloud(self, means: np.ndarray,
                                header: Header):
        if means.shape[0] == 0:
            return

        fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]

        cloud_msg = pc2.create_cloud(header, fields,
                                     means.tolist())
        self.feat_pub.publish(cloud_msg)

    # ──────────────────────────────────────────────────────────
    #  Publish normals as arrow markers (RViz)
    # ──────────────────────────────────────────────────────────

    def _publish_normal_markers(self, means: np.ndarray,
                                 normals: np.ndarray,
                                 header: Header):
        arr = MarkerArray()

        # Delete all stale markers first
        del_marker = Marker()
        del_marker.header = header
        del_marker.action = Marker.DELETEALL
        arr.markers.append(del_marker)

        arrow_len = float(self.voxel_size) * 1.5

        for i, (pm, um) in enumerate(zip(means, normals)):
            m = Marker()
            m.header    = header
            m.ns        = 'normals'
            m.id        = i
            m.type      = Marker.ARROW
            m.action    = Marker.ADD

            # Start = Pm,  End = Pm + arrow_len * um
            start = Point(x=float(pm[0]),
                          y=float(pm[1]),
                          z=float(pm[2]))
            end   = Point(x=float(pm[0] + arrow_len * um[0]),
                          y=float(pm[1] + arrow_len * um[1]),
                          z=float(pm[2] + arrow_len * um[2]))
            m.points = [start, end]

            m.scale.x = arrow_len * 0.05   # shaft diameter
            m.scale.y = arrow_len * 0.12   # head diameter
            m.scale.z = arrow_len * 0.15   # head length

            m.color = ColorRGBA(r=0.0, g=1.0, b=0.5, a=0.8)

            arr.markers.append(m)

        self.marker_pub.publish(arr)


# ──────────────────────────────────────────────────────────────
#  Entry point
# ──────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = SonarSurfaceFeatures()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()