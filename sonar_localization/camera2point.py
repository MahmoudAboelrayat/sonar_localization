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
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

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

    The fisheye model maps an angle θ = atan2(r, z) through:
        r' = k1·θ + k2·θ³ + k3·θ⁵ + k4·θ⁷
    then applies K to get pixel coords — very different from pinhole!

    Returns
    -------
    uv     : (M, 2) int    pixel coordinates of valid projected points
    depths : (M,)   float  Z depth of each valid point
    """
    # 1. Keep only points in front of the camera
    front = points_xyz[:, 2] > 0.1
    pts = points_xyz[front]

    if pts.shape[0] == 0:
        return np.zeros((0, 2), dtype=int), np.zeros(0)

    # 2. cv2.fisheye.projectPoints expects shape (N, 1, 3)
    pts_input = pts.reshape(-1, 1, 3).astype(np.float64)

    # rvec / tvec are zero because points are already in camera frame
    rvec = np.zeros((3, 1), dtype=np.float64)
    tvec = np.zeros((3, 1), dtype=np.float64)

    uv_distorted, _ = cv2.fisheye.projectPoints(
        pts_input, rvec, tvec, K, D
    )
    uv = np.round(uv_distorted.reshape(-1, 2)).astype(int)

    # 3. Keep only pixels inside the image
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
        # Fisheye / equidistant distortion coefficients
        self.declare_parameter("Camera.k1",  0.017006)
        self.declare_parameter("Camera.k2", -0.024202)
        self.declare_parameter("Camera.k3", -0.054404)
        self.declare_parameter("Camera.k4",  0.074688)

        fx = self.get_parameter("Camera.fx").value
        fy = self.get_parameter("Camera.fy").value
        cx = self.get_parameter("Camera.cx").value
        cy = self.get_parameter("Camera.cy").value
        k1 = self.get_parameter("Camera.k1").value
        k2 = self.get_parameter("Camera.k2").value
        k3 = self.get_parameter("Camera.k3").value
        k4 = self.get_parameter("Camera.k4").value

        self.K = np.array([[fx,  0, cx],
                           [ 0, fy, cy],
                           [ 0,  0,  1]], dtype=np.float64)

        # cv2.fisheye expects D as (4, 1)
        self.D = np.array([[k1], [k2], [k3], [k4]], dtype=np.float64)

        # ---- display / sync parameters ---------------------------------------
        self.declare_parameter("point_size", 3)
        self.declare_parameter("min_depth",  0.3)
        self.declare_parameter("max_depth",  5.0)
        self.declare_parameter("queue_size", 10)
        self.declare_parameter("slop",      0.1)

        self.point_size = self.get_parameter("point_size").value
        self.min_depth  = self.get_parameter("min_depth").value
        self.max_depth  = self.get_parameter("max_depth").value
        slop            = self.get_parameter("slop").value

        # ---- extrinsic: sonar(NED) → camera frame ----------------------------
        # Base rotation: NED (X=fwd, Y=right, Z=down) → camera (X=right, Y=down, Z=fwd)
        #   cam_X = NED_Y,  cam_Y = NED_Z,  cam_Z = NED_X
        # Fine-tune with RPY offsets (degrees) for physical camera mounting.
        #
        # Translation: sonar_pos - camera_pos, measured in body/NED frame (X=fwd, Y=right, Z=down)
        # Example: sonar is 0.3m ahead and 0.1m below the camera → tx=0.3, tz=0.1
        self.declare_parameter("ext.roll",  0.0)   # degrees — camera mounting correction
        self.declare_parameter("ext.pitch", 0.0)
        self.declare_parameter("ext.yaw",   0.0)
        self.declare_parameter("ext.tx",    0.0)   # metres — sonar_pos minus cam_pos, NED X (fwd)
        self.declare_parameter("ext.ty",    0.0)   # metres — sonar_pos minus cam_pos, NED Y (right)
        self.declare_parameter("ext.tz",    0.0)   # metres — sonar_pos minus cam_pos, NED Z (down)

        roll_deg  = self.get_parameter("ext.roll").value
        pitch_deg = self.get_parameter("ext.pitch").value
        yaw_deg   = self.get_parameter("ext.yaw").value
        tx_ned = self.get_parameter("ext.tx").value
        ty_ned = self.get_parameter("ext.ty").value
        tz_ned = self.get_parameter("ext.tz").value

        # NED → camera base rotation
        # cam X = NED Y (right),  cam Y = NED Z (down),  cam Z = NED X (forward)
        R_ned2cam = np.array([
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 0.0],
        ], dtype=np.float64)

        # Additional RPY fine-tune (applied after base rotation, in camera frame)
        r, p, y = np.radians([roll_deg, pitch_deg, yaw_deg])
        Rx = np.array([[1,0,0],[0,np.cos(r),-np.sin(r)],[0,np.sin(r),np.cos(r)]])
        Ry = np.array([[np.cos(p),0,np.sin(p)],[0,1,0],[-np.sin(p),0,np.cos(p)]])
        Rz = np.array([[np.cos(y),-np.sin(y),0],[np.sin(y),np.cos(y),0],[0,0,1]])
        R_total = Rz @ Ry @ Rx @ R_ned2cam

        # Translation: convert sonar-camera offset from NED to camera frame.
        # T_cam_lidar transforms a sonar-frame point into camera frame:
        #   p_cam = R_total @ p_sonar + t
        # where t = R_ned2cam @ (p_sonar_ned - p_cam_ned) = R_ned2cam @ [tx, ty, tz]
        t_ned = np.array([tx_ned, ty_ned, tz_ned], dtype=np.float64)
        t_cam = R_ned2cam @ t_ned   # convert to camera frame

        self.T_cam_lidar = np.eye(4)
        self.T_cam_lidar[:3, :3] = R_total
        self.T_cam_lidar[:3,  3] = t_cam

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
            slop=slop,
        )
        self.sync.registerCallback(self.callback)

        # ---- publisher --------------------------------------------------------
        self.pub = self.create_publisher(Image, "/projection/image", 10)

        self.get_logger().info(
            "PointCloudProjector ready  [fisheye / equidistant model]\n"
            f"  K  = fx={fx:.3f}  fy={fy:.3f}  cx={cx:.3f}  cy={cy:.3f}\n"
            f"  D  = k1={k1}  k2={k2}  k3={k3}  k4={k4}\n"
            f"  point_size = {self.point_size}  |  "
            f"depth = [{self.min_depth}, {self.max_depth}] m  |  slop = {slop} s\n"
            f"  extrinsics rpy=[{roll_deg}, {pitch_deg}, {yaw_deg}] deg  "
            f"t=[{tx_ned:.3f}, {ty_ned:.3f}, {tz_ned:.3f}] m (NED)\n"
            f"  T_cam_lidar =\n{np.array2string(self.T_cam_lidar, precision=3)}"
        )

    # ------------------------------------------------------------------
    def set_extrinsics(self, R: np.ndarray, t: np.ndarray):
        """
        Set the LiDAR→Camera extrinsic transform.
        Replace with a TF2 lookup if your frames are published.

        R : (3,3) rotation    — LiDAR frame to Camera frame
        t : (3,)  translation — LiDAR origin in Camera frame
        """
        self.T_cam_lidar = np.eye(4)
        self.T_cam_lidar[:3, :3] = R
        self.T_cam_lidar[:3,  3] = t

    # ------------------------------------------------------------------
    def callback(self, img_msg: Image, cloud_msg: PointCloud2):
        self.get_logger().info("synced")
        # ---- decode image -----------------------------------------------------
        try:
            cv_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        img_h, img_w = cv_img.shape[:2]

        # ---- decode point cloud -----------------------------------------------
        gen = pc2.read_points(cloud_msg, field_names=("x", "y", "z"),
                              skip_nans=True)
        # read_points returns a structured array with named fields (x, y, z).
        # Stack the fields into a plain (N, 3) float32 array.
        pts_raw = np.array(list(gen))                              # structured dtype
        if pts_raw.shape[0] == 0:
            self.get_logger().warn("Empty point cloud received.", throttle_duration_sec=2.0)
            self._publish(cv_img, img_msg.header)
            return
        pts = np.stack(
            [pts_raw["x"], pts_raw["y"], pts_raw["z"]], axis=1
        ).astype(np.float32)                                       # (N, 3)

        # ---- transform points into camera frame -------------------------------
        pts_h   = np.hstack([pts, np.ones((len(pts), 1), dtype=np.float32)])
        pts_cam = (self.T_cam_lidar @ pts_h.T).T[:, :3]

        # ---- project with fisheye model ---------------------------------------
        uv, depths = project_points_fisheye(
            pts_cam, self.K, self.D, img_h, img_w)

        # ---- draw projected points --------------------------------------------
        overlay = cv_img.copy()

        if len(uv) > 0:
            colors = colorize_depth(depths, self.min_depth, self.max_depth)
            r = max(1, self.point_size)
            for i in range(len(uv)):
                cx, cy = int(uv[i, 0]), int(uv[i, 1])
                color  = (int(colors[i, 0]), int(colors[i, 1]), int(colors[i, 2]))
                cv2.circle(overlay, (cx, cy), r, color, -1)

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
        """Draw a vertical JET colorbar with depth labels on the right side."""
        h, w = img.shape[:2]
        bar_h, bar_w = 200, 20
        margin = 10

        x0 = w - bar_w - margin - 40   # leave room for text
        y0 = margin

        gradient = np.linspace(255, 0, bar_h, dtype=np.uint8).reshape(bar_h, 1)
        bar_color = cv2.applyColorMap(gradient, cv2.COLORMAP_JET)   # (bar_h,1,3)
        bar_color = np.repeat(bar_color, bar_w, axis=1)              # (bar_h,bw,3)

        # Paste bar onto image
        img[y0:y0+bar_h, x0:x0+bar_w] = bar_color

        # Labels
        font      = cv2.FONT_HERSHEY_SIMPLEX
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

    # To apply LiDAR→Camera extrinsic calibration, call:
    #   R = np.array([...])   # (3,3)
    #   t = np.array([...])   # (3,)
    #   node.set_extrinsics(R, t)
    # Default is identity (LiDAR and camera share the same frame).

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()