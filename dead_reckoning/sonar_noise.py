#!/usr/bin/env python3
'''
range resolution: 0.005 m

'''

import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg    import Header

import sensor_msgs_py.point_cloud2 as pc2

def add_missing_returns(points, dropout_rate=0.3):
    mask = np.random.rand(len(points)) > dropout_rate
    return points[mask]

def compute_beam_covariance_sensor_frame(point: np.ndarray,
                                          alpha_v_rad: float,
                                          alpha_h_rad: float,
                                          eta: float) -> np.ndarray:
    r = np.linalg.norm(point)

    vertical_var = (r * np.tan(alpha_v_rad / 2.0)) ** 2
    horizontal_var = (r * np.tan(alpha_h_rad / 2.0)) ** 2
    range_var   = (eta / 2.0) ** 2

    sigma_beam = np.diag([range_var, horizontal_var, vertical_var])
    vx = np.array([1.0, 0.0, 0.0])  
    vq = np.cross(point, vx)

    vq_norm = np.linalg.norm(vq)
    if vq_norm < 1e-9:
        return sigma_beam   
    vq = vq / vq_norm

    alpha_q = -np.arccos(
        np.dot(point, vx) / (np.linalg.norm(point) + 1e-9)
    )

    q0 = np.cos(alpha_q / 2.0)
    q1, q2, q3 = vq * np.sin(alpha_q / 2.0)

    R = np.array([
        [q0**2+q1**2-q2**2-q3**2, 2*q1*q2-2*q0*q3,          2*q1*q3+2*q0*q2        ],
        [2*q1*q2+2*q0*q3,         q0**2-q1**2+q2**2-q3**2,   2*q2*q3-2*q0*q1        ],
        [2*q1*q3-2*q0*q2,         2*q2*q3+2*q0*q1,           q0**2-q1**2-q2**2+q3**2]
    ])

    sigma_sensor = R @ sigma_beam @ R.T
    return sigma_sensor

def add_3dupic_noise(points: np.ndarray,
                      alpha_v_deg: float = 1.6,
                      alpha_h_deg: float = 0.85,
                      eta: float = 0.0015) -> np.ndarray:

    alpha_v_rad = np.deg2rad(alpha_v_deg)
    alpha_h_rad = np.deg2rad(alpha_h_deg)
    noisy = np.zeros_like(points)

    for i, pt in enumerate(points):
        sigma = compute_beam_covariance_sensor_frame(pt, alpha_v_rad, alpha_h_rad, eta)
        # Sample from the Gaussian centred on the measurement
        noisy[i] = np.random.multivariate_normal(pt, sigma)

    return noisy.astype(np.float32)

def add_false_returns(points, false_rate=0.05, max_range=15.0):
    
    n_false = int(len(points) * false_rate)
    
    indices = np.random.randint(0, len(points), n_false)
    ghosts  = points[indices] + np.random.normal(0, 0.5, (n_false, 3))
    
    return np.vstack([points, ghosts])

# ──────────────────────────────────────────────────────────────
#  ROS2 Node
# ──────────────────────────────────────────────────────────────

class SonarNoiseSimulator(Node):

    def __init__(self):
        super().__init__('sonar_noise_simulator')

        # ── Subscriber ────────────────────────────────────────
        self.sub_ = self.create_subscription(
            PointCloud2,
            '/sonar/point_cloud_local',
            self.cloud_callback,
            10)

        # ── Publisher ─────────────────────────────────────────
        self.pub_ = self.create_publisher(
            PointCloud2,
            '/sonar/point_cloud_noisy',
            10)

    

    # ──────────────────────────────────────────────────────────
    #  Callback
    # ──────────────────────────────────────────────────────────

    def cloud_callback(self, msg: PointCloud2):
        # ── Convert PointCloud2 → (N, 3) numpy array ──────────
        raw = list(pc2.read_points(msg,
                                   field_names=('x', 'y', 'z'),
                                   skip_nans=True))
        if not raw:
            self.get_logger().warn('Received empty point cloud — skipping.')
            return

        points = np.array([[x, y, z] for x, y, z in raw], dtype=np.float32)

        self.get_logger().log(f'Input: {len(points)} points')

        # ── Apply noise ───────────────────────────────────────
        points = add_missing_returns(points)
        noisy = add_3dupic_noise(
            points=points
        )
        noisy = add_false_returns(noisy)

        # ── Convert back to PointCloud2 and publish ───────────
        fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]

        noisy_msg = pc2.create_cloud(msg.header, fields, noisy.tolist())
        self.pub_.publish(noisy_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SonarNoiseSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()