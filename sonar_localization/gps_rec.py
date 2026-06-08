#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import warnings
import matplotlib
matplotlib.use('Agg')  # no display needed
warnings.filterwarnings('ignore', message='Unable to import Axes3D')
import matplotlib.pyplot as plt
import numpy as np

class GPSRecorder(Node):
    def __init__(self):
        super().__init__('gps_recorder')
        self.coords = []
        qos = rclpy.qos.QoSProfile(
            depth=10,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)
        # self.create_subscription(NavSatFix, '/fix', self.cb, qos)
        self.create_subscription(NavSatFix, '/imu/nav_sat_fix', self.cb, qos)
        

    def cb(self, msg):
        if msg.status.status >= 0:
            self.coords.append((msg.latitude, msg.longitude))
            self.get_logger().info(f'Points: {len(self.coords)}')

    def save_map(self):
        if not self.coords:
            return

        lats = np.array([c[0] for c in self.coords])
        lons = np.array([c[1] for c in self.coords])

        # Convert lat/lon to Web Mercator (EPSG:3857) — required by contextily
        R = 6378137.0
        xs = np.radians(lons) * R
        ys = np.log(np.tan(np.pi / 4.0 + np.radians(lats) / 2.0)) * R

        fig, ax = plt.subplots(figsize=(8, 8))
        ax.plot(xs, ys, color='red', linewidth=2, zorder=3, label='Trajectory')
        ax.plot(xs[0],  ys[0],  'o', color='lime',      markersize=6,
                label='Start', zorder=4)
        ax.plot(xs[-1], ys[-1], 's', color='dodgerblue', markersize=6,
                label='End',   zorder=4)
        ax.legend(loc='best')

        try:
            import contextily as ctx
            # Pad axes so there is context around the trajectory
            margin = 20  # metres — increase for more surrounding area
            ax.set_xlim(xs.min() - margin, xs.max() + margin)
            ax.set_ylim(ys.min() - margin, ys.max() + margin)
            # CartoDB Positron — clean, print-friendly street map
            # Tiles are cached in ~/.cache/contextily/ after first download
            ctx.add_basemap(ax, source=ctx.providers.CartoDB.Positron, zoom='auto')
            ax.set_axis_off()
        except ImportError:
            self.get_logger().warn(
                'contextily not installed — run: pip install contextily')
            ax.set_xlabel('Easting (m)')
            ax.set_ylabel('Northing (m)')
            ax.grid(True, alpha=0.3)

        fig.tight_layout(pad=0.5)
        fig.savefig('trajectory.png', dpi=300, bbox_inches='tight')
        fig.savefig('trajectory.pdf',           bbox_inches='tight')
        plt.close(fig)
        self.get_logger().info(
            f'Saved trajectory.png / trajectory.pdf  ({len(self.coords)} points)')

def main():
    rclpy.init()
    node = GPSRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_map()
    rclpy.shutdown()

if __name__ == '__main__':
    main()