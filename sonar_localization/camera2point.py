#!/usr/bin/env python3
"""
ROS2 Node: Point Cloud Projector
Subscribes to a camera image and a LiDAR point cloud, projects the 3D points
onto the image, colorizes them by depth, and publishes the result.

Topics (defaults, remappable via parameters):
  Subscribed:
    /camera/image_raw          (sensor_msgs/Image)
    /lidar/points              (sensor_msgs/PointCloud2)
  Published:
    /projection/image          (sensor_msgs/Image)

Camera model: fisheye / equidistant (k1–k4), uses cv2.fisheye.undistortPoints.

Usage:
  ros2 run <your_pkg> pointcloud_projector.py

  # Override any camera intrinsic at launch:
  ros2 run <your_pkg> pointcloud_projector.py --ros-args \
    -p Camera.fx:=764.962868 \
    -p Camera.fy:=764.164554 \
    -p Camera.cx:=637.957857 \
    -p Camera.cy:=216.417073 \
    -p Camera.k1:=0.017006  \
    -p Camera.k2:=-0.024202 \
    -p Camera.k3:=-0.054404 \
    -p Camera.k4:=0.074688  \
    -p point_size:=3 \
    -p max_depth:=30.0

  # Update any parameter at runtime (rqt_reconfigure or CLI):
  ros2 param set /pointcloud_projector ext.pitch -- -5.0
  ros2 param set /pointcloud_projector max_depth 20.0
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rcl_interfaces.msg import SetParametersResult

import message_filters
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from cv_bridge import CvBridge

import numpy as np
import cv2


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def colorize_depth(depths: np.ndarray, min_d: float, max_d: float) -> np.ndarray:
    """Map scalar depth values → BGR color using JET colormap."""
    norm = np.clip((depths - min_d) / (max_d - min_d + 1e-6), 0.0, 1.0)
    norm_u8 = (norm * 255).astype(np.uint8)
    colors_bgr = cv2.applyColorMap(norm_u8, cv2.COLORMAP_JET)  # (N, 1, 3)
    return colors_bgr[:, 0, :]  # (N, 3)


def project_points_fisheye(
    points_xyz: np.ndarray,   # (N, 3)  in the camera frame
    K: np.ndarray,            # (3, 3)  intrinsic matrix
    D: np.ndarray,            # (4, 1)  fisheye distortion coeffs [k1,k2,k3,k4]
    img_h: int,
    img_w: int,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Project 3-D points (in camera frame) to pixel coordinates using the
    fisheye / equidistant camera model (cv2.fisheye).

    Returns
    -------
    uv     : (M, 2) int    pixel coordinates of valid projected points
    depths : (M,)   float  Z depth of each valid point
    """
    front = points_xyz[:, 2] > 0.1
    pts = points_xyz[front]

    if pts.shape[0] == 0:
        return np.zeros((0, 2), dtype=int), np.zeros(0)

    pts_input = pts.reshape(-1, 1, 3).astype(np.float64)
    rvec = np.zeros((3, 1), dtype=np.float64)
    tvec = np.zeros((3, 1), dtype=np.float64)

    uv_distorted, _ = cv2.fisheye.projectPoints(pts_input, rvec, tvec, K, D)
    uv = np.round(uv_distorted.reshape(-1, 2)).astype(int)

    inbounds = (
        (uv[:, 0] >= 0) & (uv[:, 0] < img_w) &
        (uv[:, 1] >= 0) & (uv[:, 1] < img_h)
    )
    return uv[inbounds], pts[inbounds, 2]


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class PointCloudProjector(Node):

    def __init__(self):
        super().__init__("pointcloud_projector")

        # ---- camera intrinsic parameters -------------------------------------
        self.declare_parameter("Camera.fx", 764.962868)
        self.declare_parameter("Camera.fy", 764.164554)
        self.declare_parameter("Camera.cx", 637.957857)
        self.declare_parameter("Camera.cy", 216.417073)
        self.declare_parameter("Camera.k1",  0.017006)
        self.declare_parameter("Camera.k2", -0.024202)
        self.declare_parameter("Camera.k3", -0.054404)
        self.declare_parameter("Camera.k4",  0.074688)

        # ---- display / sync parameters ---------------------------------------
        self.declare_parameter("point_size", 3)
        self.declare_parameter("min_depth",  0.3)
        self.declare_parameter("max_depth",  5.0)
        self.declare_parameter("queue_size", 10)
        self.declare_parameter("slop",       0.1)

        # ---- extrinsic: sonar(NED) → camera frame ----------------------------
        self.declare_parameter("ext.roll",  0.0)
        self.declare_parameter("ext.pitch", -2.0)
        self.declare_parameter("ext.yaw",   0.0)
        self.declare_parameter("ext.tx",    0.01)
        self.declare_parameter("ext.ty",    0.09)
        self.declare_parameter("ext.tz",    0.0)

        # ---- build initial state from declared values ------------------------
        self.point_size = self.get_parameter("point_size").value
        self.min_depth  = self.get_parameter("min_depth").value
        self.max_depth  = self.get_parameter("max_depth").value

        self._rebuild_intrinsics()
        self._rebuild_extrinsics()

        # ---- register dynamic parameter callback ----------------------------
        self.add_on_set_parameters_callback(self._on_parameters)

        # ---- state ------------------------------------------------------------
        self.bridge = CvBridge()

        # ---- QoS --------------------------------------------------------------
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=self.get_parameter("queue_size").value,
        )

        # ---- subscribers (time-synchronised) ----------------------------------
        self.sub_image = message_filters.Subscriber(
            self, Image, "/mr_pinchy/payload/camera/image_raw", qos_profile=sensor_qos)
        self.sub_cloud = message_filters.Subscriber(
            self, PointCloud2, "/sonar/point_cloud", qos_profile=sensor_qos)

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.sub_image, self.sub_cloud],
            queue_size=self.get_parameter("queue_size").value,
            slop=self.get_parameter("slop").value,
        )
        self.sync.registerCallback(self.callback)

        # ---- publisher --------------------------------------------------------
        self.pub = self.create_publisher(Image, "/projection/image", 10)

        self.get_logger().info(
            "PointCloudProjector ready  [fisheye / equidistant model]\n"
            f"  K  = fx={self.K[0,0]:.3f}  fy={self.K[1,1]:.3f}  "
            f"cx={self.K[0,2]:.3f}  cy={self.K[1,2]:.3f}\n"
            f"  D  = {self.D.flatten().tolist()}\n"
            f"  point_size={self.point_size}  depth=[{self.min_depth}, {self.max_depth}] m\n"
            f"  T_cam_lidar =\n{np.array2string(self.T_cam_lidar, precision=3)}"
        )

    # ------------------------------------------------------------------
    # Dynamic parameter callback
    # ------------------------------------------------------------------

    _EXT_PARAMS   = {'ext.roll', 'ext.pitch', 'ext.yaw', 'ext.tx', 'ext.ty', 'ext.tz'}
    _INTR_PARAMS  = {'Camera.fx', 'Camera.fy', 'Camera.cx', 'Camera.cy',
                     'Camera.k1', 'Camera.k2', 'Camera.k3', 'Camera.k4'}

    def _on_parameters(self, params):
        """Called before any parameter change is committed.
        Use p.value from the incoming list; the parameter store still holds the old value.
        """
        # Build a lookup of incoming new values
        new = {p.name: p.value for p in params}

        # Helper: return incoming value if being changed, else current stored value
        def get(name):
            return new[name] if name in new else self.get_parameter(name).value

        changed = set(new.keys())

        if changed & self._EXT_PARAMS:
            self._rebuild_extrinsics(get)
            self.get_logger().info(
                f"[ext updated] rpy=[{get('ext.roll'):.2f}, {get('ext.pitch'):.2f}, "
                f"{get('ext.yaw'):.2f}] deg  "
                f"t=[{get('ext.tx'):.3f}, {get('ext.ty'):.3f}, {get('ext.tz'):.3f}] m"
            )

        if changed & self._INTR_PARAMS:
            self._rebuild_intrinsics(get)
            self.get_logger().info(
                f"[intrinsics updated] fx={get('Camera.fx'):.3f}  fy={get('Camera.fy'):.3f}  "
                f"cx={get('Camera.cx'):.3f}  cy={get('Camera.cy'):.3f}"
            )

        if 'point_size' in new:
            self.point_size = int(new['point_size'])
            self.get_logger().info(f"[param] point_size = {self.point_size}")
        if 'min_depth' in new:
            self.min_depth = float(new['min_depth'])
            self.get_logger().info(f"[param] min_depth = {self.min_depth}")
        if 'max_depth' in new:
            self.max_depth = float(new['max_depth'])
            self.get_logger().info(f"[param] max_depth = {self.max_depth}")

        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------
    # Builder helpers — accept an optional getter so they work both at
    # init (no getter → read from parameter store) and inside the callback
    # (getter merges incoming new values with stored values).
    # ------------------------------------------------------------------

    def _rebuild_intrinsics(self, get=None):
        if get is None:
            get = lambda name: self.get_parameter(name).value
        fx = get("Camera.fx"); fy = get("Camera.fy")
        cx = get("Camera.cx"); cy = get("Camera.cy")
        k1 = get("Camera.k1"); k2 = get("Camera.k2")
        k3 = get("Camera.k3"); k4 = get("Camera.k4")
        self.K = np.array([[fx,  0, cx],
                           [ 0, fy, cy],
                           [ 0,  0,  1]], dtype=np.float64)
        self.D = np.array([[k1], [k2], [k3], [k4]], dtype=np.float64)

    def _rebuild_extrinsics(self, get=None):
        if get is None:
            get = lambda name: self.get_parameter(name).value
        roll_deg  = get("ext.roll")
        pitch_deg = get("ext.pitch")
        yaw_deg   = get("ext.yaw")
        tx_ned    = get("ext.tx")
        ty_ned    = get("ext.ty")
        tz_ned    = get("ext.tz")

        # NED → camera base rotation
        # cam X = NED Y (right),  cam Y = NED Z (down),  cam Z = NED X (forward)
        R_ned2cam = np.array([
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 0.0],
        ], dtype=np.float64)

        r, p, y = np.radians([roll_deg, pitch_deg, yaw_deg])
        Rx = np.array([[1,0,0],[0,np.cos(r),-np.sin(r)],[0,np.sin(r),np.cos(r)]])
        Ry = np.array([[np.cos(p),0,np.sin(p)],[0,1,0],[-np.sin(p),0,np.cos(p)]])
        Rz = np.array([[np.cos(y),-np.sin(y),0],[np.sin(y),np.cos(y),0],[0,0,1]])
        R_total = Rz @ Ry @ Rx @ R_ned2cam

        t_ned = np.array([tx_ned, ty_ned, tz_ned], dtype=np.float64)
        t_cam = R_ned2cam @ t_ned

        self.T_cam_lidar = np.eye(4)
        self.T_cam_lidar[:3, :3] = R_total
        self.T_cam_lidar[:3,  3] = t_cam

    # ------------------------------------------------------------------
    def callback(self, img_msg: Image, cloud_msg: PointCloud2):
        self.get_logger().info("synced")
        try:
            cv_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        img_h, img_w = cv_img.shape[:2]

        gen = pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True)
        pts_raw = np.array(list(gen))
        if pts_raw.shape[0] == 0:
            self.get_logger().warn("Empty point cloud received.", throttle_duration_sec=2.0)
            self._publish(cv_img, img_msg.header)
            return
        pts = np.stack(
            [pts_raw["x"], pts_raw["y"], pts_raw["z"]], axis=1
        ).astype(np.float32)

        pts_h   = np.hstack([pts, np.ones((len(pts), 1), dtype=np.float32)])
        pts_cam = (self.T_cam_lidar @ pts_h.T).T[:, :3]

        uv, depths = project_points_fisheye(pts_cam, self.K, self.D, img_h, img_w)

        overlay = cv_img.copy()
        if len(uv) > 0:
            colors = colorize_depth(depths, self.min_depth, self.max_depth)
            r = max(1, self.point_size)
            for i in range(len(uv)):
                cx_px, cy_px = int(uv[i, 0]), int(uv[i, 1])
                color = (int(colors[i, 0]), int(colors[i, 1]), int(colors[i, 2]))
                cv2.circle(overlay, (cx_px, cy_px), r, color, -1)

        overlay = self._draw_legend(overlay, self.min_depth, self.max_depth)
        self._publish(overlay, img_msg.header)

    # ------------------------------------------------------------------
    def _publish(self, cv_img: np.ndarray, header):
        out_msg = self.bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
        out_msg.header = header
        self.pub.publish(out_msg)

    # ------------------------------------------------------------------
    @staticmethod
    def _draw_legend(img: np.ndarray, min_d: float, max_d: float) -> np.ndarray:
        h, w = img.shape[:2]
        bar_h, bar_w = 200, 20
        margin = 10

        x0 = w - bar_w - margin - 40
        y0 = margin

        gradient = np.linspace(255, 0, bar_h, dtype=np.uint8).reshape(bar_h, 1)
        bar_color = cv2.applyColorMap(gradient, cv2.COLORMAP_JET)
        bar_color = np.repeat(bar_color, bar_w, axis=1)

        img[y0:y0+bar_h, x0:x0+bar_w] = bar_color

        font       = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.4
        thickness  = 1
        tx = x0 + bar_w + 4

        for frac, label in [(0.0, f"{max_d:.0f}m"),
                            (0.5, f"{(min_d+max_d)/2:.0f}m"),
                            (1.0, f"{min_d:.0f}m")]:
            ty = int(y0 + frac * (bar_h - 1))
            cv2.putText(img, label, (tx, ty), font, font_scale,
                        (255, 255, 255), thickness, cv2.LINE_AA)

        return img


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudProjector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
