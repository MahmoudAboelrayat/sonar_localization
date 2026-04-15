#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
from message_filters import ApproximateTimeSynchronizer, Subscriber

import numpy as np
import cv2
from cv_bridge import CvBridge
import struct

class SonarIntensityNode(Node):
    def __init__(self):
        super().__init__('sonar_intensity_node')

        # Parameters
        pc_topic  = self.declare_parameter('topics.pointcloud',    '/sonar/point_cloud').value
        img_topic = self.declare_parameter('topics.image',         '/sonar/intensity_image').value
        out_topic = self.declare_parameter('topics.output',        '/sonar/point_cloud_intensity').value
        self.fov_h = self.declare_parameter('sonar.fov_horizontal_deg', 90.0).value * np.pi / 180.0
        self.fov_v = self.declare_parameter('sonar.fov_vertical_deg',   40.0).value * np.pi / 180.0

        self.bridge = CvBridge()

        # Publisher
        self.pub = self.create_publisher(PointCloud2, out_topic, 10)

        # Synchronized subscribers
        self.pc_sub  = Subscriber(self, PointCloud2, pc_topic)
        self.img_sub = Subscriber(self, Image,       img_topic)

        self.sync = ApproximateTimeSynchronizer(
            [self.pc_sub, self.img_sub], queue_size=10, slop=0.05)
        self.sync.registerCallback(self.callback)

        self.get_logger().info(f"Listening on '{pc_topic}' and '{img_topic}'")

    # ── PointCloud2 helpers ───────────────────────────────────────────────────

    def pc2_to_xyz(self, msg: PointCloud2) -> np.ndarray:
        """Extract XYZ from PointCloud2 → (N, 3) float32 array."""
        # Parse field offsets from the message itself
        field_offsets = {f.name: f.offset for f in msg.fields}
        x_off = field_offsets['x']
        y_off = field_offsets['y']
        z_off = field_offsets['z']

        point_step = msg.point_step
        n_points   = msg.width * msg.height
        data       = np.frombuffer(msg.data, dtype=np.uint8)

        xyz = np.zeros((n_points, 3), dtype=np.float32)
        for i in range(n_points):
            base = i * point_step
            xyz[i, 0] = struct.unpack_from('f', data, base + x_off)[0]
            xyz[i, 1] = struct.unpack_from('f', data, base + y_off)[0]
            xyz[i, 2] = struct.unpack_from('f', data, base + z_off)[0]
        return xyz

    def pc2_to_xyz_fast(self, msg: PointCloud2) -> np.ndarray:
        """Faster XYZ extraction assuming standard float32 XYZ layout."""
        data = np.frombuffer(msg.data, dtype=np.uint8)
        n    = msg.width * msg.height
        step = msg.point_step

        # Reshape into (N, point_step) then slice x/y/z bytes
        data_2d = data.reshape(n, step)
        x = np.frombuffer(data_2d[:, 0:4].tobytes(), dtype=np.float32)
        y = np.frombuffer(data_2d[:, 4:8].tobytes(), dtype=np.float32)
        z = np.frombuffer(data_2d[:, 8:12].tobytes(), dtype=np.float32)
        return np.stack([x, y, z], axis=1)

    def xyzi_to_pc2(self, xyzi: np.ndarray, header) -> PointCloud2:
        """Convert (N, 4) float32 XYZI array → PointCloud2 message."""
        msg = PointCloud2()
        msg.header = header

        n_points  = xyzi.shape[0]
        itemsize  = np.dtype(np.float32).itemsize  # 4 bytes

        msg.height    = 1
        msg.width     = n_points
        msg.is_dense  = False
        msg.is_bigendian = False
        msg.point_step   = 4 * itemsize   # x y z intensity
        msg.row_step     = msg.point_step * n_points

        # Field descriptors
        from sensor_msgs.msg import PointField
        msg.fields = [
            PointField(name='x',         offset=0,            datatype=PointField.FLOAT32, count=1),
            PointField(name='y',         offset=1*itemsize,   datatype=PointField.FLOAT32, count=1),
            PointField(name='z',         offset=2*itemsize,   datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=3*itemsize,   datatype=PointField.FLOAT32, count=1),
        ]

        msg.data = xyzi.astype(np.float32).tobytes()
        return msg

    # ── Main callback ─────────────────────────────────────────────────────────

    def callback(self, pc_msg: PointCloud2, img_msg: Image):

        # ── Convert image to float32 [0, 1] ──────────────────────────────
        try:
            cv_img = self.bridge.imgmsg_to_cv2(img_msg)
        except Exception as e:
            self.get_logger().error(f'cv_bridge: {e}')
            return

        if cv_img.dtype == np.uint8:
            intensity_f = cv_img.astype(np.float32) / 255.0
        elif cv_img.dtype == np.uint16:
            intensity_f = cv_img.astype(np.float32) / 65535.0
        else:
            intensity_f = cv_img.astype(np.float32)

        # Ensure single channel
        if len(intensity_f.shape) == 3:
            intensity_f = cv2.cvtColor(intensity_f, cv2.COLOR_BGR2GRAY)

        img_h, img_w = intensity_f.shape

        # ── Extract XYZ ───────────────────────────────────────────────────
        try:
            xyz = self.pc2_to_xyz_fast(pc_msg)
        except Exception:
            # Fallback to safe parser if layout assumption fails
            xyz = self.pc2_to_xyz(pc_msg)

        # ── Filter NaN / zero points ──────────────────────────────────────
        valid = np.isfinite(xyz).all(axis=1)
        xyz   = xyz[valid]

        if xyz.shape[0] == 0:
            self.get_logger().warn('No valid points in cloud')
            return

        # ── Back-project XYZ → pixel (WaterLinked projection model) ──────
        #   Forward model:  x = r·cos(pitch)·cos(yaw)
        #                   y = r·cos(pitch)·sin(yaw)
        #                   z = -r·sin(pitch)
        #   Inverse:        yaw   = atan2(y, x)
        #                   pitch = -asin(z / r)

        r     = np.linalg.norm(xyz, axis=1)                          # (N,)
        valid_r = r > 1e-6
        xyz   = xyz[valid_r]
        r     = r[valid_r]

        yaw   =  np.arctan2(xyz[:, 1], xyz[:, 0])                    # (N,)
        pitch = -np.arcsin(np.clip(xyz[:, 2] / r, -1.0, 1.0))        # (N,)

        # ── Reject points outside FOV ─────────────────────────────────────
        in_fov = (np.abs(yaw)   <= self.fov_h / 2.0) & \
                 (np.abs(pitch) <= self.fov_v / 2.0)

        xyz   = xyz[in_fov]
        yaw   = yaw[in_fov]
        pitch = pitch[in_fov]

        if xyz.shape[0] == 0:
            self.get_logger().warn(
                'All points outside FOV — check fov_horizontal_deg / fov_vertical_deg')
            return

        # ── Map angles → pixel coordinates ───────────────────────────────
        px = np.round(
            (yaw   + self.fov_h / 2.0) / self.fov_h * (img_w - 1)
        ).astype(int)
        py = np.round(
            (pitch + self.fov_v / 2.0) / self.fov_v * (img_h - 1)
        ).astype(int)

        px = np.clip(px, 0, img_w - 1)
        py = np.clip(py, 0, img_h - 1)

        # ── Sample intensity from image ───────────────────────────────────
        intensity = intensity_f[py, px]                               # (N,)

        # ── Assemble XYZI array ───────────────────────────────────────────
        xyzi = np.column_stack([xyz, intensity])                      # (N, 4)

        # ── Publish ───────────────────────────────────────────────────────
        out_msg = self.xyzi_to_pc2(xyzi, pc_msg.header)
        self.pub.publish(out_msg)

        self.get_logger().info(
            f'Published {xyzi.shape[0]} points | image: {img_w}x{img_h}',
            throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(SonarIntensityNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()