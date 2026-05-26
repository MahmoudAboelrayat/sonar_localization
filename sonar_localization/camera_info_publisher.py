#!/usr/bin/env python3
"""
Publish camera calibration as sensor_msgs/msg/CameraInfo.

Reads calibration.json produced by cam_calb.py and publishes continuously
so any node subscribing to camera_info gets the intrinsics.

Usage:
  ros2 run <pkg> camera_info_publisher  (if installed as a node)

  # Or run directly:
  python3 camera_info_publisher.py
  python3 camera_info_publisher.py --ros-args \
      -p calibration_file:=/path/to/calibration.json \
      -p topic:=/camera/camera_info \
      -p frame_id:=camera_link \
      -p rate:=10.0
"""

import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from rcl_interfaces.msg import ParameterDescriptor


class CameraInfoPublisher(Node):
    def __init__(self):
        super().__init__('camera_info_publisher')

        self.declare_parameter('calibration_file', '/home/thor/sonar_ws/src/sonar_localization/config/calibration.json',
            ParameterDescriptor(description='Path to calibration JSON from cam_calb.py'))
        self.declare_parameter('topic', '/camera/camera_info',
            ParameterDescriptor(description='Topic to publish CameraInfo on'))
        self.declare_parameter('frame_id', 'camera_link',
            ParameterDescriptor(description='frame_id in the CameraInfo header'))
        self.declare_parameter('rate', 10.0,
            ParameterDescriptor(description='Publish rate in Hz'))

        cal_file = self.get_parameter('calibration_file').value
        topic    = self.get_parameter('topic').value
        frame_id = self.get_parameter('frame_id').value
        rate     = self.get_parameter('rate').value

        self._msg = self._load(cal_file, frame_id)

        self._pub = self.create_publisher(CameraInfo, topic, 10)
        self.create_timer(1.0 / rate, self._publish)

        self.get_logger().info(f"Publishing CameraInfo on '{topic}' at {rate} Hz")
        self.get_logger().info(
            f"  {self._msg.width}x{self._msg.height}  "
            f"fx={self._msg.k[0]:.1f}  fy={self._msg.k[4]:.1f}  "
            f"cx={self._msg.k[2]:.1f}  cy={self._msg.k[5]:.1f}"
        )

    def _load(self, path: str, frame_id: str) -> CameraInfo:
        with open(path, 'r') as f:
            cal = json.load(f)

        msg = CameraInfo()
        msg.header.frame_id = frame_id
        msg.width  = int(cal['image_width'])
        msg.height = int(cal['image_height'])

        # K: 3×3 camera matrix, row-major → 9 elements
        K = cal['camera_matrix']
        msg.k = [K[r][c] for r in range(3) for c in range(3)]

        # D: distortion coefficients, flattened
        D = cal['dist_coeffs']
        if isinstance(D[0], list):
            D = D[0]
        msg.d = [float(v) for v in D]
        msg.distortion_model = 'plumb_bob'

        # R: rectification matrix — identity for a monocular camera
        msg.r = [1.0, 0.0, 0.0,
                 0.0, 1.0, 0.0,
                 0.0, 0.0, 1.0]

        # P: 3×4 projection matrix — same as K with a zero column for monocular
        fx, fy = K[0][0], K[1][1]
        cx, cy = K[0][2], K[1][2]
        msg.p = [fx,  0.0, cx,  0.0,
                 0.0, fy,  cy,  0.0,
                 0.0, 0.0, 1.0, 0.0]

        return msg

    def _publish(self):
        self._msg.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(self._msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
