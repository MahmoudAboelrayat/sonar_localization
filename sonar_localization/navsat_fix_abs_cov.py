#!/usr/bin/env python3
"""
navsat_fix_abs_cov.py

Subscribes to a NavSatFix topic and republishes with:
  - position_covariance_type forced to COVARIANCE_TYPE_DIAGONAL_KNOWN (2)
  - diagonal covariance values taken as absolute values (guards against
    sensors that publish negative or zero variances)
  - optionally overrides all three diagonal variances with fixed fallback
    values when the incoming diagonal is zero / unknown

Useful for making GPS/GNSS messages compatible with robot_localization's
navsat_transform_node, which requires a known covariance type.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import NavSatFix


class NavsatFixAbsCov(Node):
    # NavSatFix covariance type constants
    COV_TYPE_UNKNOWN       = 0
    COV_TYPE_APPROXIMATED  = 1
    COV_TYPE_DIAGONAL      = 2
    COV_TYPE_FULL          = 3

    def __init__(self):
        super().__init__("navsat_fix_abs_cov")

        self.declare_parameter("input_topic",  "/fix")
        self.declare_parameter("output_topic", "/fix_abs")
        # Fallback sigmas [m] used when the incoming covariance diagonal is zero
        self.declare_parameter("fallback_sigma_h", 1.0)   # horizontal (lat/lon)
        self.declare_parameter("fallback_sigma_v", 2.0)   # vertical  (alt)
        # If true, always use the fallback values regardless of what the sensor reports
        self.declare_parameter("force_fallback", False)

        in_topic       = self.get_parameter("input_topic").value
        out_topic      = self.get_parameter("output_topic").value
        sig_h          = self.get_parameter("fallback_sigma_h").value
        sig_v          = self.get_parameter("fallback_sigma_v").value
        self.force_fb  = self.get_parameter("force_fallback").get_parameter_value().bool_value

        self.fallback_var_h = sig_h ** 2
        self.fallback_var_v = sig_v ** 2

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.sub = self.create_subscription(NavSatFix, in_topic,  self.callback, qos)
        self.pub = self.create_publisher(NavSatFix,    out_topic, 10)

        self.get_logger().info(
            f"navsat_fix_abs_cov: {in_topic} → {out_topic}  "
            f"fallback σ_h={sig_h:.3f} m  σ_v={sig_v:.3f} m  "
            f"force_fallback={self.force_fb}"
        )

    def callback(self, msg: NavSatFix):
        out = NavSatFix()
        out.header           = msg.header
        out.status           = msg.status
        out.latitude         = msg.latitude
        out.longitude        = msg.longitude
        out.altitude         = msg.altitude

        # Copy the full 3×3 covariance matrix
        cov = list(msg.position_covariance)

        if self.force_fb or msg.position_covariance_type == self.COV_TYPE_UNKNOWN:
            # Replace diagonal with fallback variances
            cov[0] = self.fallback_var_h   # lat–lat
            cov[4] = self.fallback_var_h   # lon–lon
            cov[8] = self.fallback_var_v   # alt–alt
        else:
            # Take absolute value of the diagonal to guard against sign errors
            for i in (0, 4, 8):
                cov[i] = abs(cov[i]) if cov[i] != 0.0 else (
                    self.fallback_var_h if i != 8 else self.fallback_var_v
                )

        out.position_covariance      = cov
        out.position_covariance_type = self.COV_TYPE_DIAGONAL

        self.pub.publish(out)


def main():
    rclpy.init()
    node = NavsatFixAbsCov()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
