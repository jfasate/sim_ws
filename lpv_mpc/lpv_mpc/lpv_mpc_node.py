#!/usr/bin/env python3
"""
LPV-MPC (Linear Parameter Varying Model Predictive Control) node for F1Tenth.

Subscribes to odometry, runs a dynamic bicycle-model MPC (adapted from
autonomous360), and publishes AckermannDriveStamped commands to follow
waypoints loaded from a CSV file.
"""

import math
import os
import time

import numpy as np
from qpsolvers import solve_qp

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

from lpv_mpc.support_files import SupportFilesF1Tenth
from lpv_mpc.utils import nearest_point


class LPVMPCNode(Node):

    def __init__(self):
        super().__init__('lpv_mpc_node')

        # ── ROS parameters ────────────���─────────────────────────────
        self.declare_parameter('map_name', 'icra')
        self.declare_parameter('csv_path', '')
        self.declare_parameter('speed_scale', 1.0)
        self.declare_parameter('odom_topic', '/ego_racecar/odom')
        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('Ts', 0.02)
        self.declare_parameter('hz', 10)
        self.declare_parameter('m', 3.47)
        self.declare_parameter('Iz', 0.04712)
        self.declare_parameter('Cf', 90.0)
        self.declare_parameter('Cr', 110.0)
        self.declare_parameter('lf', 0.15875)
        self.declare_parameter('lr', 0.17145)
        self.declare_parameter('mju', 0.015)
        self.declare_parameter('Q_diag', [10.0, 500.0, 100.0, 100.0])
        self.declare_parameter('S_diag', [10.0, 500.0, 100.0, 100.0])
        self.declare_parameter('R_diag', [50.0, 5.0])
        self.declare_parameter('qp_solver', 'cvxopt')

        map_name = self.get_parameter('map_name').value
        csv_path_param = self.get_parameter('csv_path').value
        self.speed_scale = self.get_parameter('speed_scale').value
        odom_topic = self.get_parameter('odom_topic').value
        drive_topic = self.get_parameter('drive_topic').value
        self.qp_solver = self.get_parameter('qp_solver').value

        Ts = self.get_parameter('Ts').value
        hz = self.get_parameter('hz').value

        # Build support-class params from ROS parameters
        support_params = {
            'Ts': Ts, 'hz': hz,
            'm': self.get_parameter('m').value,
            'Iz': self.get_parameter('Iz').value,
            'Cf': self.get_parameter('Cf').value,
            'Cr': self.get_parameter('Cr').value,
            'lf': self.get_parameter('lf').value,
            'lr': self.get_parameter('lr').value,
            'mju': self.get_parameter('mju').value,
            'Q_diag': list(self.get_parameter('Q_diag').value),
            'S_diag': list(self.get_parameter('S_diag').value),
            'R_diag': list(self.get_parameter('R_diag').value),
        }

        # ── Support class (vehicle model + MPC matrices) ───────────
        self.support = SupportFilesF1Tenth(support_params)
        self.constants = self.support.constants
        self.Ts = self.constants['Ts']
        self.hz = self.constants['hz']
        self.inputs = self.constants['inputs']
        self.outputs = self.constants['outputs']

        # ── Load waypoints ──────────────────���───────────────────────
        if csv_path_param:
            csv_file = csv_path_param
        else:
            csv_file = os.path.join(
                os.path.abspath(os.path.join('src', 'csv_data')),
                map_name + '.csv')
        self.get_logger().info(f'Loading waypoints from: {csv_file}')

        # CSV columns: s_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2; ...
        self.waypoints = np.loadtxt(csv_file, delimiter=';', skiprows=0)
        self.n_waypoints = self.waypoints.shape[0]
        self.wp_xy = self.waypoints[:, 1:3]            # (N, 2) for nearest-point
        self.wp_vx = self.waypoints[:, 5].copy()        # speed [m/s]
        self.wp_s = self.waypoints[:, 0].copy()          # arc-length

        self.wp_psi = self.waypoints[:, 3].copy()       # yaw [rad]
        # Unwrap yaw so it's continuous (no ±pi jumps)
        self.wp_psi = np.unwrap(self.wp_psi)

        # Average waypoint spacing for lookahead index computation
        self.ds = np.mean(np.diff(self.wp_s))

        # Minimum reference speed: ensure scaled speeds stay above the
        # dynamic model's stability threshold (1.5 m/s) with some margin.
        # Derived from CSV data so it adapts to any track / speed_scale.
        self.min_ref_speed = max(self.wp_vx.min() * self.speed_scale, 2.0)

        self.get_logger().info(
            f'Loaded {self.n_waypoints} waypoints, avg spacing={self.ds:.4f} m, '
            f'speed range [{self.wp_vx.min():.1f}, {self.wp_vx.max():.1f}] m/s, '
            f'min_ref_speed={self.min_ref_speed:.2f} m/s')

        # ── MPC state ────────────────────────────────────────────��─
        self.states = np.zeros(6)   # [x_dot, y_dot, psi, psi_dot, X, Y]
        self.U1 = 0.0               # current steering angle (delta)
        self.U2 = 0.0               # current acceleration (a)
        self.du = np.zeros((self.inputs * self.hz, 1))
        self.state_received = False
        self.iteration = 0

        # Lap timing
        self.prev_wp_idx = 0
        self.nr_laps = 0
        self.lap_start_time = None
        self.lap_crossed_half = False

        # ── ROS pub/sub ───────────────���──────────��──────────────────
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, 1)

        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 1)

        self.vis_waypoints_pub = self.create_publisher(Marker, '/lpv_mpc/waypoints', 1)
        self.vis_ref_pub = self.create_publisher(Marker, '/lpv_mpc/ref_traj', 1)
        self.vis_pred_pub = self.create_publisher(Marker, '/lpv_mpc/pred_path', 1)

        # Publish waypoints once
        self._publish_waypoints_marker()

        # Control timer at 1/Ts Hz
        self.control_timer = self.create_timer(self.Ts, self.control_loop)

        self.get_logger().info(
            f'LPV-MPC node started  |  Ts={self.Ts}s  hz={self.hz}  '
            f'solver={self.qp_solver}  speed_scale={self.speed_scale}')

    # ───────────��──────────────────────────────��─────────────────────
    #  Odometry callback — extract state from sim
    # ────────────────────────────────────────────────────────────────
    def odom_callback(self, msg):
        """Store latest vehicle state from the simulator odometry."""
        pose = msg.pose.pose
        twist = msg.twist.twist

        X = pose.position.x
        Y = pose.position.y

        # Quaternion → yaw
        q = pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        # Body-frame velocities (the sim already provides these in body frame)
        x_dot = twist.linear.x
        y_dot = twist.linear.y
        psi_dot = twist.angular.z

        self.states = np.array([x_dot, y_dot, yaw, psi_dot, X, Y])
        self.state_received = True

    # ────────────────────────────────────────────────────────────────
    #  Main control loop (timer callback)
    # ───────────────────────────────────────────���────────────────────
    def control_loop(self):
        if not self.state_received:
            return

        states = self.states.copy()

        # Ensure minimum forward velocity for the dynamic model.
        # The dynamic bicycle model is numerically unstable (forward-Euler)
        # below ~1.2 m/s for F1Tenth params.  Wait until the car is fast
        # enough, sending an open-loop speed command in the meantime.
        if states[0] < 1.5:
            self._publish_drive(self.U1, 2.0)
            return

        hz = self.hz  # local copy (stays constant for closed track)

        # ── 1. Find nearest waypoint ───────────────────��────────────
        _, _, _, wp_idx = nearest_point(
            np.array([states[4], states[5]]), self.wp_xy)

        # ── Lap timing ──────────────────────────────────────────────
        self._update_lap_timing(wp_idx)

        # ── 2. Build reference vector for the horizon ──────────────
        r = self._build_reference(states, wp_idx, hz)

        # ── 3. Linearize & build QP ───────────────────────────���────
        Ad, Bd, Cd, Dd = self.support.state_space(states, self.U1, self.U2)

        x_aug_t = np.array([[states[0]], [states[1]], [states[2]],
                            [states[3]], [states[4]], [states[5]],
                            [self.U1], [self.U2]])

        Hdb, Fdbt, Cdb, Adc, G, ht = self.support.mpc_simplification(
            Ad, Bd, Cd, Dd, hz, x_aug_t, self.du)

        ft = np.concatenate((x_aug_t.flatten(), r)) @ Fdbt

        # ── 4. Solve QP ────────────────────────────────────────────
        # Ensure Hessian is symmetric and positive definite
        Hdb = 0.5 * (Hdb + Hdb.T)
        eigvals = np.linalg.eigvalsh(Hdb)
        if eigvals.min() < 1e-8:
            Hdb += (1e-6 - eigvals.min()) * np.eye(Hdb.shape[0])

        try:
            du_sol = solve_qp(Hdb, ft, G, ht, solver=self.qp_solver)
            if du_sol is None:
                if self.iteration % 50 == 0:
                    # Check which constraints are infeasible at du=0
                    slack = ht - G @ np.zeros(G.shape[1])
                    n_violated = np.sum(slack < 0)
                    self.get_logger().warn(
                        f'QP infeasible: {n_violated} constraints violated at du=0  '
                        f'states={np.round(states, 3)}  U1={self.U1:.4f} U2={self.U2:.4f}  '
                        f'min_slack={slack.min():.4f}  eig_min={eigvals.min():.6f}')
                self._publish_drive(self.U1, states[0])
                return
            self.du = du_sol.reshape(-1, 1)
        except Exception as e:
            if self.iteration % 50 == 0:
                self.get_logger().warn(f'QP exception: {e}  states={np.round(states, 3)}')
            self._publish_drive(self.U1, states[0])
            return

        # ── 5. Update inputs ──────────────────────────��─────────────
        self.U1 += self.du[0][0]   # steering angle
        self.U2 += self.du[1][0]   # acceleration

        # Clamp steering and acceleration to stay within constraint bounds
        max_steer = 0.4189
        self.U1 = np.clip(self.U1, -max_steer, max_steer)
        self.U2 = np.clip(self.U2, -3.0, 3.0)

        # Compute desired speed
        speed_cmd = states[0] + self.U2 * self.Ts
        speed_cmd = max(speed_cmd, 0.0)

        # ── 6. Publish drive ─────────────────────���──────────────────
        self._publish_drive(self.U1, speed_cmd)

        # ── 7. Visualization ────────────────────────────────────────
        self._publish_ref_marker(r, hz)

        # ── 8. Logging ─────────────────────────────────────────────
        self.iteration += 1
        if self.iteration % 50 == 0:
            self.get_logger().info(
                f'[iter={self.iteration}] wp={wp_idx}  '
                f'v={states[0]:.2f} m/s  steer={math.degrees(self.U1):.1f}deg  '
                f'accel={self.U2:.2f}  speed_cmd={speed_cmd:.2f}')

    # ────────────────────────────────────��───────────────────────���───
    #  Reference trajectory builder
    # ─────────────────────────────────────────────────���──────────────
    def _build_reference(self, states, wp_idx, hz):
        """Build the reference signal vector r for the MPC horizon.

        r = [x_dot_ref_1, psi_ref_1, X_ref_1, Y_ref_1, ...,
             x_dot_ref_hz, psi_ref_hz, X_ref_hz, Y_ref_hz]

        Advances the reference based on predicted travel distance at each
        step (speed * Ts), so the reference matches where the car will
        actually be — not a fixed 1-waypoint-per-step which overshoots
        on turns.
        """
        speed = max(states[0], 1.5)
        current_psi = states[2]

        # Small lookahead offset so MPC sees just ahead of nearest point
        lookahead_dist = speed * self.Ts * 2  # ~2 timesteps ahead
        lookahead_indices = max(1, int(round(lookahead_dist / self.ds)))

        # Distance the car travels per MPC step
        dist_per_step = speed * self.Ts
        # How many waypoint indices that corresponds to
        indices_per_step = dist_per_step / self.ds

        r = np.zeros(self.outputs * hz)
        for k in range(hz):
            # Advance proportional to predicted travel distance
            advance = lookahead_indices + k * indices_per_step
            idx = (wp_idx + int(round(advance))) % self.n_waypoints
            ref_vx = max(self.wp_vx[idx] * self.speed_scale, self.min_ref_speed)
            ref_psi = self.wp_psi[idx]

            # Adjust psi reference to be close to current psi (avoid 2*pi jumps)
            while ref_psi - current_psi > np.pi:
                ref_psi -= 2.0 * np.pi
            while ref_psi - current_psi < -np.pi:
                ref_psi += 2.0 * np.pi

            r[self.outputs * k + 0] = ref_vx      # x_dot ref
            r[self.outputs * k + 1] = ref_psi      # psi ref
            r[self.outputs * k + 2] = self.wp_xy[idx, 0]  # X ref
            r[self.outputs * k + 3] = self.wp_xy[idx, 1]  # Y ref

        return r

    # ────────────────────��──────────────────────────��────────────────
    #  Lap timing
    # ─────────────────────────────────────────────��──────────────────
    def _update_lap_timing(self, wp_idx):
        if self.lap_start_time is None:
            self.lap_start_time = time.perf_counter()

        half = self.n_waypoints // 2
        if half * 0.4 < wp_idx < half * 1.6:
            self.lap_crossed_half = True

        if (self.lap_crossed_half
                and wp_idx < self.n_waypoints * 0.05
                and self.prev_wp_idx > self.n_waypoints * 0.9):
            lap_time = time.perf_counter() - self.lap_start_time
            self.get_logger().info(
                f'========== LAP {self.nr_laps} FINISHED  |  time: {lap_time:.2f}s ==========')
            self.nr_laps += 1
            self.lap_start_time = time.perf_counter()
            self.lap_crossed_half = False

        self.prev_wp_idx = wp_idx

    # ───────────────────────────��────────────────────────────────────
    #  Publishing helpers
    # ────────────────────────────────────────────────────────────────
    def _publish_drive(self, steering, speed):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.steering_angle = float(steering)
        msg.drive.speed = float(speed)
        self.drive_pub.publish(msg)

    def _publish_waypoints_marker(self):
        m = Marker()
        m.header.frame_id = 'map'
        m.type = Marker.POINTS
        m.color.g = 0.75
        m.color.a = 1.0
        m.scale.x = 0.05
        m.scale.y = 0.05
        m.id = 0
        for i in range(self.n_waypoints):
            m.points.append(Point(
                x=float(self.wp_xy[i, 0]),
                y=float(self.wp_xy[i, 1]),
                z=0.1))
        self.vis_waypoints_pub.publish(m)

    def _publish_ref_marker(self, r, hz):
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.type = Marker.LINE_STRIP
        m.color.b = 0.9
        m.color.a = 1.0
        m.scale.x = 0.06
        m.id = 1
        for k in range(hz):
            m.points.append(Point(
                x=float(r[self.outputs * k + 2]),
                y=float(r[self.outputs * k + 3]),
                z=0.2))
        self.vis_ref_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = LPVMPCNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
