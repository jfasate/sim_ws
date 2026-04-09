#!/usr/bin/env python3
"""
Cross-track error monitor for LPV-MPC.
Subscribes to odom, computes distance to nearest waypoint on the reference trajectory,
and prints statistics after each lap.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import os
import time


class CrossTrackMonitor(Node):
    def __init__(self):
        super().__init__('cross_track_monitor')

        # Load waypoints
        csv_file = os.path.join('/sim_ws/src/csv_data', 'icra.csv')
        wps = np.loadtxt(csv_file, delimiter=';', skiprows=0)
        self.wp_xy = wps[:, 1:3]
        self.n_wp = len(self.wp_xy)
        self.get_logger().info(f'Loaded {self.n_wp} waypoints from {csv_file}')

        self.errors = []
        self.lap_errors = []
        self.lap_count = 0
        self.crossed_half = False
        self.prev_idx = 0
        self.started = False
        self.start_time = None

        self.sub = self.create_subscription(
            Odometry, '/ego_racecar/odom', self.odom_cb, 10)

    def odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        pos = np.array([x, y])

        # Nearest waypoint distance
        dists = np.linalg.norm(self.wp_xy - pos, axis=1)
        min_idx = np.argmin(dists)
        error = dists[min_idx]

        # Wait until car is moving (speed > 1.0)
        vx = msg.twist.twist.linear.x
        if not self.started:
            if vx > 1.0:
                self.started = True
                self.start_time = time.time()
                self.get_logger().info('Car moving, starting measurement')
            else:
                return

        self.errors.append(error)
        self.lap_errors.append(error)

        # Lap detection
        half = self.n_wp // 2
        if half * 0.4 < min_idx < half * 1.6:
            self.crossed_half = True

        if (self.crossed_half
                and min_idx < self.n_wp * 0.05
                and self.prev_idx > self.n_wp * 0.9):
            self.lap_count += 1
            lap_time = time.time() - self.start_time
            errs = np.array(self.lap_errors)
            self.get_logger().info(
                f'\n{"="*60}\n'
                f'  LAP {self.lap_count} COMPLETE  |  time: {lap_time:.2f}s\n'
                f'  Cross-track error:\n'
                f'    Mean:  {errs.mean():.4f} m\n'
                f'    Max:   {errs.max():.4f} m\n'
                f'    Std:   {errs.std():.4f} m\n'
                f'    < 0.2m: {100*np.mean(errs < 0.2):.1f}%\n'
                f'    < 0.3m: {100*np.mean(errs < 0.3):.1f}%\n'
                f'    < 0.5m: {100*np.mean(errs < 0.5):.1f}%\n'
                f'{"="*60}')
            self.lap_errors = []
            self.crossed_half = False
            self.start_time = time.time()

        self.prev_idx = min_idx

        # Periodic status
        if len(self.errors) % 500 == 0:
            errs = np.array(self.errors[-500:])
            self.get_logger().info(
                f'[samples={len(self.errors)}] mean_err={errs.mean():.4f}m  '
                f'max_err={errs.max():.4f}m  wp={min_idx}  v={vx:.2f}')


def main():
    rclpy.init()
    node = CrossTrackMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
